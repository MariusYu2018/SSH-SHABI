import numpy as np 
import math
class SG_time_computation():
    def add_noise(self,sigma_v,sigma_d):
        v_=v_mess+3*sigma_v 
        d_add_noise = d_mess-3*sigma_d
        return v_,d_add_noise
    def time_reach_distance(self,distance,v_max):
        distance = distance-3  #这两行非常重要，表明了取了标准正太分布中速度v_的极值
        if(distance<0):
            print('Negative distance can not be used')
        if(v_<v_max):
            t_max = (v_max-v_)/max_acc_
            d_max= t_max*(v_max+ v_)/2  #add the noise 3 delta to the velocity of agent
            if(distance<d_max):
                arg=[max_acc_/2,v_,-1*distance]
                list_t_agent=[]
                list_t_agent=np.roots(arg)
                t=max(list_t_agent)
                if(t<0):
                    print('No solution for the equation exist')
            t=t_max+(distance-d_max)/v_max
            print(float(t))
            return float(t)
    def time_reach_distance_limited_jerk(self,distance,v_max, current_acc, jerk_limit):
        v_=v_mess+3
        distance=distance-3   #这两行也非常重要，对距离采取了取标准正太分布的极值，与上面函数开头采取的方法一致。
        if(distance<0):
            print('Negative distance can not be used')
        t_a_max=(max_acc_-current_acc)/jerk_limit  #time to reach the maximal velocity
        v_a_max =t_a_max*(max_acc_+current_acc)/2+v_
        if(v_a_max>v_max):
            arg=[jerk_limit/2,current_acc,v_-v_max]
            list_t_ego=[]
            list_t_ego=np.roots(arg)
            t=max(list_t_ego)
            if t<0:
                print('No solution for the ego_t exists')
            t_non_zero_jerk=float(t)
            print(t_non_zero_jerk)
            t_zero_jerk_non_zero_accl=0
        else:
            t_non_zero_jerk=t_a_max
            t_zero_jerk_non_zero_accl=(v_max-v_a_max)/max_acc_
        d_non_zero_jerk=(1/6)*jerk_limit*pow(t_non_zero_jerk,3)+0.5*current_acc*pow(t_non_zero_jerk,2)+v_*t_non_zero_jerk
        if(d_non_zero_jerk>distance):
            param_jerk=[(1/6)*jerk_limit,0.5*current_acc,  v_, -1*distance]
            list_jerk=[]
            list_jerk=np.roots(param_jerk)
            t = max(list_jerk)
            print(float(t))
            return float(t)
            
        dist_non_zero_jerk=(1/6)*jerk_limit*pow(t_non_zero_jerk,3)+0.5*current_acc*pow(t_non_zero_jerk,2)+v_*t_non_zero_jerk
        d_zero_jerk_non_zero_accl = t_zero_jerk_non_zero_accl * (v_max + v_a_max) / 2 #calculate distance of accelerated movement by average velocity times to delta time
        if (distance - dist_non_zero_jerk < d_zero_jerk_non_zero_accl):
            list_t_unenough_distance_acc=[]
            params=[max_acc_ / 2, v_a_max, -1 * (distance - dist_non_zero_jerk)]
            list_t_unenough_distance_acc=np.roots(params)
            t =max(list_t_unenough_distance_acc)
            if (t < 0):
                print("no solution for the equation")
            print(float(t_non_zero_jerk+t))
            return float(t_non_zero_jerk) + float(t)
        t = t_non_zero_jerk + t_zero_jerk_non_zero_accl + (distance - dist_non_zero_jerk - d_zero_jerk_non_zero_accl) / v_max
        print(t)
        return float(t)     #以上两个函数的定义都是参考agent.cpp中的11-71行
    
v_mess=10
d_mess=10
v_max=20
max_acc_ = 2
list_noise_1=[]
list_noise_2=[]
SG = SG_time_computation()
list_noise_1=SG.add_noise(1,1)
v_ = list_noise_1[0]
d_add_noise = list_noise_1[1]
t_agent = SG.time_reach_distance(d_add_noise,20)
max_acc_ = 3
v_mess= 10
d_mess = 40
SG.add_noise(1,1)
list_noise_2=SG.add_noise(1,1)
v_ = list_noise_2[0]
d_add_noise = list_noise_2[1]    
t_ego_leave_time = SG.time_reach_distance_limited_jerk(d_add_noise,20,1,1)
t_SG = -(t_agent - t_ego_leave_time)
print('we can get the gap time:',t_SG)


    


#the following lines are test parameter, it should be annotated when submitting
#
        
    
    


    #def comupute_safe_SG_time(sigma_d,sigma_v):
        #SG_time