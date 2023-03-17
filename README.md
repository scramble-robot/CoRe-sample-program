# CoRE2023 サンプル機体 プログラム
## 使用しているソフト
STM32CubeIDE:https://www.st.com/ja/development-tools/stm32cubeide.html

## デフォルト状態

## 操作方法
1. バッテリーの電源を入れる
2. バッテリーのスイッチをONにする
3. コントローラの電源をONにする
4. 非常停止スイッチを解除する

## 起こりそうな不具合＆解決法
- コントローラで足回りを動かそうとしても動かない
  - /Core/Src/tim.cの下記箇所の数字を増やす
  - なお、あまり増やすとCAN送信周期が下がるので制御性が低下する
  - また、制御周期が変わるので、**装填やリロードにも影響が出る**ので注意
```c
htim10.Init.Prescaler = 3600-1;
htim10.Init.Period = 10;
```
- 装填時にディスクが落ちないことがある（連続して起こる場合）
  - 装填のストロークが足りない、/Core/Src/main.cの下記部分を修正する
  - ① 装填時間を長くする
  - ② 速度を上げる
```c
if (htim == &htim10){
    ...（略）...
}else if(launcherFlag==0){
    if(underReloadState==1){
        if(ReloadState==0){
            if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14)){
                v[5]=-3000;//② 引く速度
                ReloadState=1;
            }else{
                v[5]=3000;//② 押し出す速度
            }
        }else if(ReloadState>=700){//① 移動時間
            v[5]=0;
            underReloadState=2;
        }else if(ReloadState>=1){
            v[5]=-4500;//② 引く速度
            ReloadState++;
        }
    }else{
        v[5]=0;
    }
}
```
- リロードのストロークが足りない
  - /Core/Src/main.cの下記部分を修正する
  - ① 装填時間を長くする
  - ② 速度を上げる
```c
if(v[7]!=0){
    countLR++;
    if(countLR>6000){//① 移動時間
        countLR=0;
        v[7]=0;
        launcherFlag=0;
    }
}
...（略）...
if((v[6]==0)&&(mode==move)&&(v[5]==0)){
    if(launcherLR!=rc.sw2){
        launcherLR=rc.sw2;
        launcherFlag=1;
        if(launcherLR==left){
            v[7]=1200;//② 移動速度
        }else{
            v[7]=-1100;//② 移動速度
        }
    }
}
```
- リロードと装填が絡まる
  - 装填が完全に下がりきっていないと干渉するため発生
  - 装填の時間を長く＆速くし確実に後ろに下げる
- ディスクが完全に装填から射出部分に落とさないと、リロードが引っかかる
  - リロードするときは、マガジンにディスクがない状態にしてから行う（完全に打ち切るまで行わない）
- 右後ろ足回りカップリングが外れる
