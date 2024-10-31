function next_state = unicycle(t,theta_k,u_k)

L = 0.0370;
R = 0.0300;
            next_state = [
                (R/2)*cos(theta_k)*u_k(1) + (R/2)*cos(theta_k)*u_k(2);
                (R/2)*sin(theta_k)*u_k(1) + (R/2)*sin(theta_k)*u_k(2);
                R/(2*L)*u_k(1) - R/(2*L)*u_k(2)];
            
            
            
end

