//
// パイピコ　モーションコントローラ
//
// PiPco Motion Control for 6 axes
//
//
//
// 産業用としてモーションコントローラ、およびCNCコントローラは
// サーボモーターやステッピングモーターとともに現代では必須の技術です。
// しかし、産業用としては商品性や秘匿性から内部のコードを見ることはできません。
// ホビー用として十分成功しているMach3のようなソフトでも同様です。
//
// オープンソースのCNCコントローラとしてはLinuxCNCやgrbl、3Dプリンタ用としてはMarlinがあり
// それぞれ発展しています。
//
// LinuxCNCほど巨大なコードでもなく、grblほど簡易でもない、それでいて
// CNCコントローラがどのように機能しているか？をコードを読みながら理解しやすい
// プログラムが欲しかったことから自分のCNCコントローラ作りは始まりました。
//　市販されている産業用と比べると機能は貧弱ですが、
//  一般的な加工やホビー用であれば使えるものにしたいと考えています。
//
//
// MainLoop.cpp
// 
//MainLoop.cpp  メインループ、コマンドとGコードのブロックをシリアル通信で送受信   
// |
// |          
// |
// +--- MachineControl.cpp  機械情報の処理
//       |          サイクルスタートON/OFF、フィードホールドON/OFF、オーバーライド%
//       |          機械原点、リミットスイッチの状態を
//       |          ※原点復帰のようなGコードでは規定されない動作など
//       |          ※グローバルオブジェクトで情報を共有
//       |
//       +-- G-CodeInterpriter.cpp
//       |          Gコード解析、ブロックの実行開始、ブロックの実行ループ、ブロックの終了処理
//       |          Gコードを受け取るとそのブロックをタイマー割り込みにより開始（各種補間）
//       |　　　    ※Gコードを受信してServoControl.cppに渡すデータを演算
//       |
//       +-- ServoControl.cpp
//       |          サーボに関係する処理（サーボリセット）
//       |          移動量、移動速度の計算
//       |
//       +------ PulseControl.c        
//                  タイマー割り込みにより移動残があればパルス列を出力
//                  MainLoop.cよりタイマー割り込み初期化で実行される
//                  ServoControl.c で設定された速度に合わせてパルス幅を変化させる
//                  ※マイコンの違いはここで吸収
//
//            
//
//        |
//        +------ Encoder.c         エンコーダーに関する処理
//        |
//
//　　　　　※コードの再利用を考えてグローバル変数はできるだけ減らす
//　　　　　　　　　　　　→分割したコード間のデータはデータ受け渡し専用の関数を使う
//                       →servoはオブジェクト化にしてmotionオブジェクトの下に置くべき？
//
//司令コマンド（<>で囲まれる）
//
//司令一覧
//<MOVE:0:0:0:0:0:0>
//<SET_SPEED:100>
//<SET_FEED:100>
//<SET_RAPID_FEED:100>
//<R_EN> READ ENCODER
//<R_MC> READ MACHINE COODINATE
//<R_MS> READ STOP COODINATE
//<R_RD> READ REMAINING TRAVEL DISTANCE
//<SERVO_POWER_ON>
//<SERVO_POWER_OFF>
//<SERVO_INIT>
//<RESET_X>
//<RESET_Y>
//<RESET_Z>
//<RESET_A>
//<RESET_B>
//<RESET_C>
//<CYST>
//<FHLD>
//<FEED_ABORT>
//<SET_OVERRIDE_0>
//<SET_OVERRIDE_+10>
//<SET_OVERRIDE_-10>
//<SET_FEED_RATE_0>
//<SET_FEED_RATE_+10>
//<SET_FEED_RATE_-10>
//
//""で囲まれテキストはGコードとして処理
//例:
//"G90G54G1X100.000F1000"

#include "MainLoop.h"




extern void output_coodinate (int);
extern void encoder_pin_init (void);	// Assuming this is defined elsewhere
extern void reload_encoder (void);	// Assuming this is defined elsewhere
extern void check_stop_in (void);
extern void check_emergency (void);
extern void exampleUsage (void);


void exampleUsage (void);


#define MAX_LENGTH 256
#define MAX_COMMAND_LENGTH 256
#define STOP_IN        27	//原点リミットスイッチ、タッチプローブ信号
#define EMERGENCY_STOP 28

// 最大で対応する軸数を決める（例：6軸）
#define MAX_AXES 6


//Gコードインタプリタ
#define cod_G53 0
#define cod_G54 1
#define cod_G55 2
#define cod_G56 3
#define cod_G57 4
#define cod_G58 5
#define cod_G59 6


int m_1 = 0;

char ReceiveCommand[MAX_LENGTH];	// Buffer for received data

char command_f;





MachineControl machine;
PulseControl pulse;
GCodeInterpreter gcodeInterpreter;


//===============  割り込みによりメインループとは別に実行される処理 ===========
//
// 第一コアと第二コアで別々に起動している。
// 一応は処理分散が目的だが特に意味はない^^
// コア数や処理の具合に応じて変えても良い。
//
repeating_timer_t timer_for_gcode;
repeating_timer_t timer_for_plsout;


bool
pulse_train_output (repeating_timer_t * timer)
{
  pulse.PlsOut ();
  return true;
}

bool
G_code_block_callback (repeating_timer_t * timer)
{
  if (gcodeInterpreter.block_exec_status != 0)
    {
      gcodeInterpreter.gcode_callback ();
    }

  return true;
}

//==========================================================================










//====================== メインループ =======================================
//
// メインループの主な機能はコマンドを受信し、
// そのコマンドをMachineControlに引き渡すこと。
// コマンドの解析はMachineControl以下で処理される。
//
//========= 第二コアで実行するスレッド
void
core1_entry ()
{
  command_f = 0;

  add_repeating_timer_us (10, pulse_train_output, NULL, &timer_for_plsout);
  while (true)
    {
      int count = tud_cdc_available ();
      if (count > 0)
	{
	  if (count > MAX_LENGTH - 1)
	    count = MAX_LENGTH - 1;	// Reserve space for null terminator
	  if (command_f == 0)
	    {
	      tud_cdc_read (ReceiveCommand, count);
	      command_f = 1;
	      ReceiveCommand[count] = '\0';	// Null terminate the string
	    }



	  // ここで、メインコアにデータを受信したことを伝える
	  multicore_fifo_push_blocking (1);	// シグナルを送信
	}
    }
}


//============== 第一コアで実行するスレッド
int
main ()
{

  stdio_init_all ();
  set_sys_clock_khz (130000, true);



  multicore_launch_core1 (core1_entry);

  // Initialize servo and encoder pins

  //pulse.PulsePin_init ();
  encoder_pin_init ();

  // Gコードのタイマーを設定
  add_repeating_timer_ms (100, G_code_block_callback, NULL, &timer_for_gcode);	// 例えば100msごとに呼び出す




  machine.setRapidFeedRate (10000);
  machine.setFeedRate (1000);


  while (true)
    {



      //=======  MachineControlへ受信データをリレー(コマンドに応じた処理)
      // 第二コアからのシグナルを待つ
      if (multicore_fifo_rvalid ())
	{

	  //
	  uint32_t signal = multicore_fifo_pop_blocking ();
	  if (signal == 1)
	    {			// データを受信したことを示すシグナル
	      // コマンド処理
	      machine.processCommand (ReceiveCommand);
	      machine.processCommand2 (ReceiveCommand);

	      machine.setFeedRate (ReceiveCommand);
	      memset (ReceiveCommand, 0, sizeof (ReceiveCommand));
	      command_f = 0;
	    }
	}






      //テスト用
      exampleUsage ();



      //=======  MachineControl更新処理（コマンドによらない処理）
      //エンコーダー情報更新
      //外部からのIO情報との同期
      //サーボ動作
      //アラームチェックとアラーム送信
      reload_encoder ();
      //check_stop_in ();
      //check_emergency ();



    }
  return 0;
}





// Example usage and setting up the timer
void
exampleUsage (void)
{

// block_exec_status ブロック実行管理
// 0:移動完了（次ブロック読込可）
// 1:次ブロックセット完了
// 2:ブロック実行中
  if (gcodeInterpreter.block_exec_status == 0)
    {
      switch (m_1)
	{
	case 0:
	  //Gコードセット
	  gcodeInterpreter.parseGCode ("\"G1X0Y0Z10F1000\"");
	  break;
	case 1:
	  gcodeInterpreter.parseGCode ("\"X10Y10\"");
	  break;
	case 2:
	  gcodeInterpreter.parseGCode ("\"X30Y20\"");
	  break;
	case 3:
	  gcodeInterpreter.parseGCode ("\"X10Y10\"");
	  break;
	case 4:
	  gcodeInterpreter.parseGCode ("\"G80\"");
	  break;


	}


      if (m_1 < 4)
	{
	  m_1++;
	}
      else
	{
	  m_1 = 0;
	}

    }


}
