# -*- coding: utf-8 -*-

import string

import sympy
import sympybotics

q = sympybotics.robotdef.q
pi = sympy.pi

''' walkerX rightLimb model '''
walkerX_rightLimb = sympybotics.RobotDef('walkerX rightLimb', # robot name
                                        [(0, 0, 0, q),  # list of tuples with Denavit-Hartenberg parameters
                                        ( -pi/2, 0, 0, q+pi/2),
                                        ( -pi/2, 0, 0.215, q),
                                        ( -pi/2, -0.0248, 0, q),
                                        ( pi/2, 0.0248, 0.2055, q+pi/2),
                                        ( -pi/2, 0, 0, q+pi/2),
                                        ( pi/2, 0, 0, q)], # (alpha, a, d, theta)
                                        dh_convention='modified' # either 'standard' or 'modified'
                                        )

''' set the friction model, including Coulomb and viscous '''
walkerX_rightLimb.frictionmodel = {'Coulomb', 'viscous'}    #include 'Coulomb', 'viscous' and 'offset'

''' set the motor rotational inertia '''
walkerX_rightLimb.driveinertiamodel = 'simplified'          # can be None or 'simplified'


''' get the all link dynamic parameters for identification,
    and write to the file '''
DynParms_S = walkerX_rightLimb.dynparms()

filename = open("/home/meihui/work_ubt/walkerX/dynamics/modeling/dynamic_code/DynParms.txt",'w')
for i in range(0, 7):
    for j in range(0, len(DynParms_S)/7):
        filename.write(str(DynParms_S[i * (len(DynParms_S)/7) + j]))
        filename.write(' ')
    filename.write('\n')
filename.close()

#生成动力学符号运算代码
RobotDynCode_walker_rightLimb = sympybotics.RobotDynCode(walkerX_rightLimb, verbose=True)

#输出机器人正运动学矩阵
Matrix_T=RobotDynCode_walker_rightLimb.geo.T[-1]
# print(Matrix_T)

#输出机器人雅可比矩阵
Matrix_J=RobotDynCode_walker_rightLimb.kin.J[-1]
# print(Matrix_J)

###############
#输出C代码到TXT
###############

# 参数：目标编程语言，代码，函数输出，函数名，机器人模型


# 逆动力学代码输出
Invdyn_code_walkerX = sympybotics.robotcodegen.robot_code_to_func('C',
                                                                RobotDynCode_walker_rightLimb.invdyn_code,
                                                                'Torque_out',
                                                                'Cal_Inverse_Dynamics',
                                                                walkerX_rightLimb)


filename = open("/home/meihui/work_ubt/walkerX/dynamics/modeling/dynamic_code/Cal_Inverse_Dynamics.txt",'w')

filename.write(Invdyn_code_walkerX)

filename.close()

# 惯量矩阵（M矩阵）代码输出
M_code_walkerX = sympybotics.robotcodegen.robot_code_to_func('C',
                                                            RobotDynCode_walker_rightLimb.M_code ,
                                                            'M_Matrix_out',
                                                            'Cal_Inertia_Matrix',
                                                            walkerX_rightLimb)

#
filename = open("/home/meihui/work_ubt/walkerX/dynamics/modeling/dynamic_code/Cal_Inertia_Matrix.txt",'w')

filename.write(M_code_walkerX)

filename.close()

# 科氏矩阵（C矩阵）代码输出
C_code_walkerX = sympybotics.robotcodegen.robot_code_to_func('C',
                                                            RobotDynCode_walker_rightLimb.C_code ,
                                                            'C_Matrix_out',
                                                            'Cal_Coriolis_Matrix',
                                                            walkerX_rightLimb)

filename = open("/home/meihui/work_ubt/walkerX/dynamics/modeling/dynamic_code/Cal_Coriolis_Matrix.txt",'w')

filename.write(C_code_walkerX)

filename.close()

# 科氏项代码输出
c_code_walkerX = sympybotics.robotcodegen.robot_code_to_func('C',
                                                            RobotDynCode_walker_rightLimb.c_code ,
                                                            'c_Term_out',
                                                            'Cal_Coriolis_Term',
                                                            walkerX_rightLimb)


filename = open("/home/meihui/work_ubt/walkerX/dynamics/modeling/dynamic_code/Cal_Coriolis_Term.txt",'w')

filename.write(c_code_walkerX)

filename.close()

# 重力项代码输出
G_code_walkerX = sympybotics.robotcodegen.robot_code_to_func('C',
                                                            RobotDynCode_walker_rightLimb.g_code ,
                                                            'G_Term_out',
                                                            'Cal_Gravity_Term',
                                                            walkerX_rightLimb)


filename = open("/home/meihui/work_ubt/walkerX/dynamics/modeling/dynamic_code/Cal_Gravity_Term.txt",'w')

filename.write(G_code_walkerX)

filename.close()

# 摩擦力项代码输出
F_code_walkerX = sympybotics.robotcodegen.robot_code_to_func('C',
                                                            RobotDynCode_walker_rightLimb.f_code ,
                                                            'F_Term_out',
                                                            'Cal_Friction_Term',
                                                            walkerX_rightLimb)


filename = open("/home/meihui/work_ubt/walkerX/dynamics/modeling/dynamic_code/Cal_Friction_Term.txt",'w')

filename.write(F_code_walkerX)

filename.close()

# 基本惯量参数（Base Parameters）代码输出
RobotDynCode_walker_rightLimb.calc_base_parms()
BaseParms_walkerX = RobotDynCode_walker_rightLimb.dyn.baseparms


filename = open("/home/meihui/work_ubt/walkerX/dynamics/modeling/dynamic_code/BaseParms.txt",'w')

for i in range(0, len(BaseParms_walkerX)):
    filename.write(str(BaseParms_walkerX[i]))
    filename.write('\n')


filename.close()

#要先计算Base Parameters，才能计算Hb矩阵
# 回归矩阵的独立子矩阵（Hb矩阵）代码输出
Hb_code_walkerX = sympybotics.robotcodegen.robot_code_to_func('C',
                                                                RobotDynCode_walker_rightLimb.Hb_code ,
                                                                'Hb_Term_out',
                                                                'Cal_Hb_Regressor_Matrix',
                                                                walkerX_rightLimb)


filename = open("/home/meihui/work_ubt/walkerX/dynamics/modeling/dynamic_code/Cal_Hb_Regressor_Matrix.txt",'w')

filename.write(Hb_code_walkerX)

filename.close()
# print(RobotDynCode_walker_rightLimb.dyn.Kd)



print('Finish')
