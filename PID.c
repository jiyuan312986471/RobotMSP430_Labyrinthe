#define PID_Uint struct pid_uint 
PID_Uint 
{ 
  int U_kk; 
  int ekk; 
  int ekkk; 
  int Ur; //限幅输出值,需初始化 
  int Un; //不灵敏区 
  //int multiple; //PID系数的放大倍数，用整形数据的情况下，提高PID参数的设置精度   固定为256 
  int Kp; //比例，从小往大调 
  int Ti; //积分，从大往小调 
  int Td; //微分，用巡线板时设为0 
  int k1; // 
  int k2; 
  int k3; 
}; 



/********************************************************************  
函 数 名：void Init_PID_uint(PID_uint *p) 
功    能：初始化PID参数 
说    明：调用本函数之前，应该先对Kp,Ti,Td做设置 ,简化了公式 
入口参数：PID单元的参数结构体 地址 
返 回 值：无 
***********************************************************************/ 
void Init_PID_uint(PID_Uint *p) 
{ 
  p->k1 = (p->Kp)+(p->Kp)*1024/(p->Ti)+(p->Kp)*(p->Td)/1024; 
  p->k2 = (p->Kp)+2*(p->Kp)*(p->Td)/1024; 
  p->k3 = (p->Kp)*(p->Td)/1024; 
}



/********************************************************************  
函 数 名：void reset_Uk(PID_Uint *p) 
功    能：初始化U_kk,ekk,ekkk 
说    明：在初始化时调用，改变PID参数时有可能需要调用 
入口参数：PID单元的参数结构体 地址 
返 回 值：无 
***********************************************************************/ 
void reset_Uk(PID_Uint *p) 
{ 
  p->U_kk = 0; 
  p->ekk  = 0; 
  p->ekkk = 0; 
}



/********************************************************************  
函 数 名：int PID_commen(int set,int mesure,PID_Uint *p) 
功    能：通用PID函数 
说    明：求任意单个PID的控制量 
入口参数：期望值，实测值，PID单元结构体 
返 回 值：PID控制量 
***********************************************************************/ 
int PID_common(int set, int mesure, PID_Uint *p) 
{ 
  int ek, U_k = 0; 
  ek = mesure-set; 
  if((ek>(p->Un)) || (ek<-(p->Un))) //积分不灵敏区 
    U_k = (p->U_kk)+(p->k1)*ek-(p->k2)*(p->ekk)+(p->k3)*(p->ekkk); 
  p->U_kk = U_k; 
  p->ekkk = p->ekk; 
  p->ekk = ek; 
  if(U_k > (p->Ur))                         //限制最大输出量， 
    U_k = p->Ur; 
  if(U_k < -(p->Ur)) 
    U_k = -(p->Ur); 
  return U_k/1024;  
}