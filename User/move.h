#ifndef _MOVE_H_
#define _MOVE_H_

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


/*********************************************************************************
  *@  name      : move.c
  *@  version   : 1.0.3
  *@  update log:
  1.0 2021.11.6:第一版库，实现了基础功能，引入看板娘铃兰，尚未进行完整测试，规划了下一步迭代方向
  1.0.1 2021.11.12：成功在全向轮底盘上运行自动路径规划，修复了一些bug，增加了加速度规划，调整了控制逻辑（弯道不进行障碍检测和立刻抄近路检测）
  1.0.2 2021.11.19：能够考虑固定障碍，修复了速度规划的bug（只是加入系数，留一点冗余），引入了路径拟合算法，考虑下一步迭代
  1.0.3 2021.12.24：进一步封装，准备实际上机测试
*********************************************************************************/
#include "math.h"
#include "stdlib.h"
#include "user_task.h"
#include "Resolve.h"

//结构体声明区
typedef struct heap_node
{
    short x;
    short y;
    short last_x;
    short last_y;
    float cost;
    float cost_current;
}heap_node;



//宏定义区
#define CLAMP(x, lower, upper) (x >= upper ? upper : (x <= lower ? lower : x))
#define deadzone 0.4//路径点死区大小
#define acceleration_limit_increase 1.2
#define acceleration_limit_decrease 1.2
#define acceleration_limit_turn 1
#define speed_limit 1.5
#define control_period 0.01

//全局变量区
extern Ort speed_plan[1500];
extern Ort pos_plan[1500];
extern float acceleration_plan[1500][2];//加速度计划数组，第一位为x方向加速度，第二位为y方向加速度
extern check_point *check_point_head;
extern barrier *barrier_head;
extern Ort current_pos;
extern Ort current_speed;
//extern float speed_st[2500];
extern int flag_current_path_change;
extern int flag_final_goal_change;
extern int turn_end_time;
extern double dT;
extern Ort output;
extern int flag_center_access;

//函数声明区
void move_execute(Ort current__final_goal);
void pre_plan(Ort pos_Goal);
void add_barrier(Ort pos,double range,int barrier_id);
void update_barrier(int barrier_ID,Ort pos,double range);
void remove_barrier(int barrier_ID);
int check_barrier(Ort pos1,Ort pos2);
check_point* static_path_planning(Ort start_coordinate,Ort current__final_goal);
check_point* dynamic_path_planning(Ort pos1,Ort pos2,int barrier_id);
barrier* find_barrier(int barrier_ID);
extern float Pid_Run(PID_T *Pid, float Target, float Feedback);
void shift(heap_node *root[], int i, int len);
void add_node(heap_node *root[], int len, heap_node *item);
heap_node *out_node(heap_node *root[], int len);
double cal_average(uint8_t a[5000][2],int head,int end);
Ort coordinate_transform(Ort realtive_pos);
Ort evaluate_approach_pos(int target_ID);

#endif
