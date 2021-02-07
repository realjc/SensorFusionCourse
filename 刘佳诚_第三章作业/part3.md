### 题目1

对于线特征残差：
$$d_\epsilon = \frac{|(\tilde{p_i}-p_b)\times(\tilde{p_i}-p_a)|}{|p_b-p_b|}$$

残差对位姿的雅可比$J_\epsilon = \frac{\partial d_\epsilon}{\partial T}= \frac{\partial d_\epsilon}{\partial \tilde{p_i}}\frac{\partial \tilde{p_i}}{\partial T}$

而位置对平移的雅可比：$\frac{\partial \tilde{p_i}}{\partial t} = I$
对旋转的雅可比：$\frac{\partial \tilde{p_i}}{\partial R} = -(Rp_i)^\land$
残差对位置的雅可比：
$\frac{\partial d_\epsilon}{\partial \tilde{p_i}} = \frac{1}{|p_a-p_b|}(\frac{\partial (\tilde{p_i}-p_b)^\land(\tilde{p_i}-p_a)}{\partial \tilde{p_i}}+ \frac{(\tilde{p_i}-p_b)^\land \partial(\tilde{p_i}-p_a)}{\partial \tilde{p_i}}) \\
= \frac{1}{|p_a-p_b|} ((\tilde{p_i}-p_a)^\land +(\tilde{p_i}-p_b)^\land) \\
= \frac{(p_a-p_b)^\land}{|p_a-p_b|}$
所以：
残差对平移的雅可比：$\frac{\partial J_\epsilon}{\partial t} =\frac{\partial d_\epsilon}{\partial \tilde{p_i}} \frac{\partial \tilde{p_i}}{\partial t}  = \frac{(p_a-p_b)^\land}{|p_a-p_b|} $
对旋转的雅可比：
$\frac{\partial J_\epsilon}{\partial R} =\frac{\partial d_\epsilon}{\partial \tilde{p_i}} \frac{\partial \tilde{p_i}}{\partial R}  = -\frac{(p_a-p_b)^\land}{|p_a-p_b|} (Rp_i)^\land$

对于面特征残差：
$$d_H = (\tilde{p_i}-p_i)\frac{(p_l-p_j)\times(p_m-p_j)}{|(p_l-p_j)\times(p_m-p_j)|}$$
同样的：
残差对位姿的雅可比$J_H = \frac{\partial d_H}{\partial T}= \frac{\partial d_H}{\partial \tilde{p_i}}\frac{\partial \tilde{p_i}}{\partial T}$

而残差对位置的雅可比：
$\frac{\partial d_H}{\partial \tilde{p_i}} =\frac{(p_l-p_j)\times(p_m-p_j)}{|(p_l-p_j)\times(p_m-p_j)|}$
所以：
残差对平移的雅可比：$\frac{\partial J_H}{\partial t} =\frac{\partial d_H}{\partial \tilde{p_i}} \frac{\partial \tilde{p_i}}{\partial t}  = \frac{(p_l-p_j)\times(p_m-p_j)}{|(p_l-p_j)\times(p_m-p_j)|}$
对旋转的雅可比：
$\frac{\partial J_H}{\partial R} =\frac{\partial d_H}{\partial \tilde{p_i}} \frac{\partial \tilde{p_i}}{\partial R}  = -\frac{(p_l-p_j)\times(p_m-p_j)}{|(p_l-p_j)\times(p_m-p_j)|}(Rp_i)^\land$

### 题目2
参考FLOAM， 解析求导代码如下：
线特征
```cpp
bool EdgeAnalyticCostFunction::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    
    Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);
    Eigen::Vector3d lp;
    lp = q_last_curr * curr_point + t_last_curr; //new point
    Eigen::Vector3d nu = (lp - last_point_a).cross(lp - last_point_b);
    Eigen::Vector3d de = last_point_a - last_point_b;

    residuals[0] = nu.x() / de.norm();
    residuals[1] = nu.y() / de.norm();
    residuals[2] = nu.z() / de.norm();

    if(jacobians != NULL)
    {
        if(jacobians[0] != NULL)
        {
            Eigen::Matrix3d skew_lp = skew(lp);
            Eigen::Matrix<double, 3, 6> dp_by_so3;
            dp_by_so3.block<3,3>(0,0) = -skew_lp;
            (dp_by_so3.block<3,3>(0, 3)).setIdentity();
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
            J_se3.setZero();
            Eigen::Vector3d re = last_point_b - last_point_a;
            Eigen::Matrix3d skew_re = skew(re);

            J_se3.block<3,6>(0,0) = skew_re * dp_by_so3/de.norm();
            
        }
    }

    return true;
 
}   
```
面特征
```cpp

bool SurfNormAnalyticCostFunction::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Map<const Eigen::Quaterniond> q_w_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_w_curr(parameters[0] + 4);
    Eigen::Vector3d point_w = q_w_curr * curr_point + t_w_curr;

    residuals[0] = plane_unit_norm.dot(point_w) + negative_OA_dot_norm;

    if(jacobians != NULL)
    {
        if(jacobians[0] != NULL)
        {
            Eigen::Matrix3d skew_point_w = skew(point_w);

            Eigen::Matrix<double, 3, 6> dp_by_so3;
            dp_by_so3.block<3,3>(0,0) = -skew_point_w;
            (dp_by_so3.block<3,3>(0, 3)).setIdentity();
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
            J_se3.setZero();
            J_se3.block<1,6>(0,0) = plane_unit_norm.transpose() * dp_by_so3;
   
        }
    }
    return true;

}   
```
### 题目3
