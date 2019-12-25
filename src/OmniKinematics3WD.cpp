#include "OmniKinematics3WD.h"

void OmniKinematics3WD::getOutput(int x, int y, int yaw, double yawAngle, int pwm[3])
{
  XVector = x;
  YVector = y;
  YawVector = yaw;     //ユーザーからの回転量
  yawAngle = yawAngle; //ロボットの傾き

  //逆運動学を使って各軸の移動量からモータの回転方向・量を計算する
  if (YawVector != 0)
  { //ユーザーからの回転命令があるとき
    userBias = yawAngle;
    pwm[0] = int(-double(XVector * cos(yawAngle * DEG_TO_RAD)) - double(YVector * sin(yawAngle * DEG_TO_RAD)) - YawVector);
    pwm[1] = int(+double(XVector * cos(double(yawAngle + 60.0) * DEG_TO_RAD)) + double(YVector * sin(double(yawAngle + 60.0) * DEG_TO_RAD)) - YawVector);
    pwm[2] = int(-double(XVector * cos(double(yawAngle + 120.0) * DEG_TO_RAD)) - double(YVector * sin(double(yawAngle + 120.0) * DEG_TO_RAD)) - YawVector);
  }
  else
  { //IMUがYAW軸を自動補正するとき
    double outputYawPWM = double(double(yawAngle - userBias) * 7);
    if (-3.5 < outputYawPWM && outputYawPWM < 3.5)
    {
      outputYawPWM = 0;
    }
    pwm[0] = int(-double(XVector * cos(yawAngle * DEG_TO_RAD)) - double(YVector * sin(yawAngle * DEG_TO_RAD)) - outputYawPWM);
    pwm[1] = int(+double(XVector * cos(double(yawAngle + 60.0) * DEG_TO_RAD)) + double(YVector * sin(double(yawAngle + 60.0) * DEG_TO_RAD)) - outputYawPWM);
    pwm[2] = int(-double(XVector * cos(double(yawAngle + 120.0) * DEG_TO_RAD)) - double(YVector * sin(double(yawAngle + 120.0) * DEG_TO_RAD)) - outputYawPWM);
  }

  //計算上の最大出力を求める
  int max = 0;
  for (int i = 0; i < 3; i++)
  {
    if (max < abs(pwm[i]))
      max = abs(pwm[i]);
  }

  //最大出力が許容最大出力を超えていたら再計算する
  double rate = 0;
  if (maxAllocateOutput < max) //0 < maxAllocateOutputより割り算しても大丈夫。rateは0ではない数の除算により算出されるので0に成り得ない。
  {
    rate = double(double(max) / maxAllocateOutput);
    for (int i = 0; i < 3; i++)
    {
      pwm[i] = int(double(pwm[i]) / rate); //安全な割り算
    }
  }
}