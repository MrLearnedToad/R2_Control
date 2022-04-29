#ifndef _ANN_H_
#define _ANN_H_

#include "main.h"

/*据本人统计，在引入二次元以后，bug出现频率下降了11.4514%
           ..   ...       rX  ji.                                                                   
            .              .:    ..                                                                 
            Y    ......     .7. ...::::.                                                            
            7.   ......:.    .i. .   ..i::                                                          
            .i   .:.......     :.       . :7i                                                       
             7    .:...r:      .::        .. :i                                                     
             i     ::....        .:.       .. 7v::..::iirir:.                                       
             i.     ....           :ri..         .:i:     .::::::.                                  
              7   ....irv7r:       .i:...     .:i:.           .  .:::.                              
              L.    ..:ii7i.     .:7r.       .:.              :.    ..ii     . .                    
              .s      .....:77. .7L:.       UKs:iiriii:i:..    .     :  :v:...is:i                  
               ri   .::iii:rri iYr:.       ZZZJ..:.:.::::iiii:.:.    .v  ..   .. ......  r5viY      
               .v      .:ii:i:r7.:.       gZbU:.:::.........:::ii:.   :J   :...      ...iEQBBM   ...
             ..:      .iiii:i1i.:.      .BD2s1sv7v77rrii::::...:. .:.  v:   ..    .YbvrX2QBq.    .:i
                .LJ:..ii:::r2:.:.   .  .J:   .       :i .:::i::.. :.::::j     .:7IEXv:7QBu  ..:.::r.
               ..:...:... :u:..: .     s.   :        i      .:rri:.:.:.iB: .  :BMBBIPBDB:           
              .iJ  ..... .v::.i .     i:   ::        :        . .ir7r.. Eu  .  rBQBEQBB.            
               :.  .... .r:i.:. :  . .L    7         :        .  . .7U7.7b   .  XBBQBB       :.::iii
       .      7....:.   7i:i.: .. .  ri.  r:  .     .:  .     .  ..   ssPE   .   BB75   .    .i:... 
  ::..::     .:.:X7u  .uri:i.i r     7.. .D. ..  .  ::     . ...  .   . iB.    . 7Rr   :ri..   .... 
.    ..         i77 :rPvrir:i. r  . :r.  JS  ::.. . 1.. .   . .. ..   .  Ii     . 7:i. .rrvr:.:.  . 
     i       .r.7LirrvL:7:iir. r .  Jr. .q7  i. .. .1..  . .  .. :.      :r  .  .  .rrri..:i:       
    i       .v  7:vvrU:rv.:rr .i.  .P:. L.7  s.    :s.. . . . .  :. . :   i     :. .iiirr.....      
   :i       .u.:sis  i:v7.:7r  r   :ui  . .  Y.  . r7..  . .  :  :   .:  ...    .: .r:::ii          
   vi        Ji:r.v::.r2r iri .L.  .vEBBggZJ::: . .r:.: . .. .s .7. ..: .: :    :.: :i .... ... .:r.
  ..         ii:77 :.i15: :vi .I.  7BBusqPXgBM..7.:. :r  . . 77 .S . .: .r i.   :::. L. ..   :u7.   
: .         v.ir:r.: UMr: :rr  sr .gI :BBS21D2i  :i  :1. ... v: ru   :. .7 i.   ri r  7 ....J  .    
 .       .:.X::7.iQ. vSi: .r7  rU  .r Z7i  BB.       ..:. .. :  rr   j  .S :..  rv v. :i .. :       
v.       i ..u.::ii . si: :.Y. .Ii  v r    :P           . .:i1Ji .  :j  :Q .... rL :5  J. .. i      
i       i  ..ur:i   : rL..:.7.  75. .. .....              .BQBBIQD  Ur  rB ...  i5  B. .U.:ii:ri    
        :   irs1rri.:  v:.::.:  .JX. :                   :BB7vuDiBBiD:  SQ .:.  :B. sP  LY          
       ::      ::Ri:   r:: I r.. iJg  .                  Ui. vBB  BUU.  2I:... ..Bi .qi  7          
       L.        Sii   v.: 7:.. . iP1                    .  ..vr  QgY  i7Jr... ..ri  YJ  ri     .7. 
    .. 7:         i1r  1::  v . .. YS                      ..:   .7B:  U.sr: .  i... 1iL .7      r:.
    .L .:         r7vvrP:.  .:   .. g:  .                        :jj  Si ri. .  v  : vi7  i      :  
     i:           :7rL.jr.:  :.   : .B:         .               r1U. Y5 :S.  ...v  i r7vS ri     :. 
.     r            1i..YP i.  :    : 2Zi                       rUi: DB  Z.   ..::  5 :K.g.:i     .J 
r.    ::  .         r   g..:   7   .  IuU                    i7777..2  E7   ...i   B :2 7::r     .7 
 ..   i.             ii .v :i  .u     ::   i             .:rjb  .  .  5K    :...  :g .D.  7:     :. 
  i   i:         ...  .:.K. 2:  i7   .   JZQUvi:::::r....I1i:iiv  .  7D    .. :  s:2 :d   I.    :u  
   :i i:          ...   :vi qQ:  iS  .. QBBBBBBBBQBBBRg   :Irjir7  .rv.   .. :2.S7:Y ur   I   ii.   
   .us::          .   .:  b .BBi  51  : iBBBgQEBBPEbQQBr  .i.:rv: :g::   ... 1vrri:S j. .: i:r.     
     i:.:           .SKIPdQL SgQZ  Yg .  ruPggggBQDgBBBb      rLrrLj:   :.Y  qrii:YQr.    .J.       
      r...   ..    .DQgMEPKB. QBBB..BBi .Y:   .JQBBBBBg    .:j .2:g7  .: vi .Qir::5v.             ..
      :: ....     7bBEqSBPPBBiEBEi:.:QB .sP:iPBQBBYIBB      Yu  : 7  :. 7E  ui  .:                ..
. ..  72...  . ..BBBBQMbZQQBBYi:      i .5.s:rr:   i7.    . BU i    i.:IrZ .S  :                  .i
. .:. :vv7r:i: :EBDgQRRQBBK    7:     . :riIu.    7:v    rB YB:Bi :i:7Y .. L: r       .           ir
. .... Lrrr7i::BBPSREDQBB7      .. .. :.i  JM5   .dBPjvSvv5ZDBQBP:::    r :i L.  .  ..          .7ui
:..... vvrriri.DBQRBBBBB    ......... Si. .:DBBBBB5BBQgS7..:ii7:. .i:. :..rr7   ..... .  ..   .:vv7.
i::. ..s2:rrriJi 75BQBBK ........... 7v   XUuu:sY K2    ..:i::.:..    rLiS5r:.. .....:.... .iviiri:.
7:iv777:Yi:iisr       1. .............   jU rrP1M i:                 i1r.  u5: .:iiri......ir:irriri
ri7X     7v7sr       :7  .............  LBRBERBu:                  :vj.   rQdU..:77:   .:...rv7vv77r
7:7ru.    gQ2        Y: .............. iBBBZKQP:                 rUi.:   v. .7rj2: .:ir7r i.v7Li:7vi
i:irrJ7  :1R        iL:..................7SJMB.    i::.iL    ..iiiii:s  J2.    .7q:YYJr7i r: . : :7:
r:ii:LgL  Pr        2v:................    .i7Z7..rr..:.rr:ir.  :  ..7r:.:PBr      :ij    .i.  vi 7:
r:i:2u:u  5:       :gv::............:.....   ri         .        .         iQqY      .i2:i:::r77rrr:
r:iP5...:Id   .   .dYL7i:.:::.......:ii:... :7   :7:          JP.:    :vr.. sUZI7IDDgEQBBZXjvriirrr:
i.v.   .DBB.     .11irrii:..::.:.:....:iii.:.iiS:. ::.  rY:SBBQBRri.ir:.       vUKu.       .:ri:.::.
███████╗██╗   ██╗███████╗██╗   ██╗██████╗  █████╗ ███╗   ██╗    ██╗███████╗     ██████╗ ██╗   ██╗██████╗     ██╗     ██╗ ██████╗ ██╗  ██╗████████╗
██╔════╝██║   ██║╚══███╔╝██║   ██║██╔══██╗██╔══██╗████╗  ██║    ██║██╔════╝    ██╔═══██╗██║   ██║██╔══██╗    ██║     ██║██╔════╝ ██║  ██║╚══██╔══╝
███████╗██║   ██║  ███╔╝ ██║   ██║██████╔╝███████║██╔██╗ ██║    ██║███████╗    ██║   ██║██║   ██║██████╔╝    ██║     ██║██║  ███╗███████║   ██║   
╚════██║██║   ██║ ███╔╝  ██║   ██║██╔══██╗██╔══██║██║╚██╗██║    ██║╚════██║    ██║   ██║██║   ██║██╔══██╗    ██║     ██║██║   ██║██╔══██║   ██║   
███████║╚██████╔╝███████╗╚██████╔╝██║  ██║██║  ██║██║ ╚████║    ██║███████║    ╚██████╔╝╚██████╔╝██║  ██║    ███████╗██║╚██████╔╝██║  ██║   ██║   
╚══════╝ ╚═════╝ ╚══════╝ ╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═══╝    ╚═╝╚══════╝     ╚═════╝  ╚═════╝ ╚═╝  ╚═╝    ╚══════╝╚═╝ ╚═════╝ ╚═╝  ╚═╝   ╚═╝    
                                                                   
*/

#define multiply_factor_P 1.2//调节神经网络kP输出大小
#define multiply_factor_I 0.2//调节神经网络kI输出大小
#define multiply_factor_D 0.2//调节神经网络kD输出大小
#define multiply_factor_PID_out 1//调节PID输出大小
#define normalize_factor 5//调节神经网络输入时数据归一化程度
#define ANN_learning_rate 0.05//调节神经网络学习速率
/*求绝对值*/
#define ABS(x) (((x) > 0) ? x : (-(x)))
/*限幅*/
#define CLAMP(x, lower, upper) (x >= upper ? upper : (x <= lower ? lower : x))

/*PID 数据结构体*/
typedef struct
{
    float Error;
    float Error_Last;
    float Error_Sum;

    float KP;
    float KI;
    float KD;

    float I_Limit;
    float I_Coefficient;
    float Dead_Zone;

    float P_OUT;
    float I_OUT;
    float D_OUT;
    float PID_OUT;

    float I_MAX;
    float PID_MAX;

} PID_T;

typedef struct matrix
{
    double mat[35];
    int row;
    int col;
}matrix;

typedef struct NN_handler
{
    matrix input_2_hidden;
    matrix hidden_2_output;
    double inputsum[5];
    double hiddensum[5];
    double hiddenout[5];
    double outputout[5];
    double inputout[5];
    int input_num;
    int hidden_num;
    double last_speed;
    char a;
}NN_handler;

typedef struct ANN_PID_handle
{
    NN_handler network;
    PID_T pid_handle;
    double error[5];
    double input[3];
}ANN_PID_handle;

extern uint8_t NNlearn;

//外部调用函数
double ANN_pid_run(ANN_PID_handle *handle,double target,double current_value);
ANN_PID_handle* ANN_pid_init(ANN_PID_handle *handle);
float Pid_Run(PID_T *Pid, float Target, float Feedback);
//内部调用函数
static void NN_bcak_prop(NN_handler *network,double *output_error,double rate);
static void NN_fprop(NN_handler *network,double *input);
static NN_handler* NN_create_network(int input_num,int hidden_num,NN_handler *network);

#endif
