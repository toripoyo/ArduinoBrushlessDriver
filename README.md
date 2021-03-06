Arduino nano を使い、3相ブラシレスモーターを駆動する基板とプログラムです。

## 基板
/KiCad ディレクトリ下に、KiCadプロジェクト一式が保存されています。  
　回路図：Inverter.pdf  
　部品表：PartsList.xlsx  

## 制御プログラム
ArduinoBrushlessDriver下に、Arduinoプロジェクト一式が保存されています。

## 基板の動作
1. 3相ブラシレスモーターをUVW出力端子に接続し、基板に12Vを供給します
1. ボリューム（RV1）を最小位置から1/4ほどの位置にします
1. SW1を押すと、モーターが強制回転され、その後センサレス駆動がはじまります
1. RV1を調整すると、出力電圧が調整でき、回転速度を変えることができます

## モーターに合わせた調整
* 強制回転の際にうまく回らない場合は、RV1を回し、印可電圧を上げて試してみてください。（上げすぎに注意）
* 強制回転を終了する回転数を調整したい場合は、プログラム179行目の定数の値を書き換えてください
