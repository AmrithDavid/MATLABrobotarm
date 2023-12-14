function [ ] = UR3move( )
clf;

%#ok<*NASGU>

robot = UR3;


workspace = [-1 1 -1 1 -1 1]; % Set the size of the workspace when drawing the robot
scale = 1;
q = zeros(1,6); % Create a vector of initial joint angles
robot.model.plot(q,'workspace',workspace,'scale',scale); % Plot the robot

R_down = trotx(pi/2);  


liftBehindPose = transl(-0.5,0.5, 0.5) * R_down;
pickBehindPose = transl(-0.5, 0.5, 0) * R_down;
liftForwardPose = transl(-0.5, 0.5, 0.3) * R_down; 
frontPose = transl(-0.5, -0.4, 0.2) * R_down; 
placePose = transl(-0.5, -0.4, 0) * R_down;
originalPose = robot.model.fkine(q).T; 
posesSequence = {liftBehindPose, pickBehindPose, liftForwardPose, frontPose, placePose, originalPose};

qMatrix = q;
deltaT = 0.05; % time step
epsilon = 0.1; 

 

interpolation_steps = 20; % number of intermediate steps

for k = 1:length(posesSequence)-1 
    T_start = robot.model.fkine(qMatrix(end,:)).T; 

    for interp_step = 1:interpolation_steps 
        alpha = interp_step / interpolation_steps; 
        T_intermediate = trinterp(T_start, posesSequence{k+1}, alpha);

        T = robot.model.fkine(qMatrix(end,:)).T; 

        Rd = T_intermediate(1:3,1:3);
        Ra = T(1:3,1:3);
        Rdot = (1/deltaT)*(Rd - Ra); 
        S = Rdot*Ra';

        deltaX = T_intermediate(1:3,4) - T(1:3,4); 
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)]; 

        xdot = [linear_velocity;angular_velocity]; 
        J = robot.model.jacob0(qMatrix(end,:));  

        m = sqrt(det(J*J')); % closer to 0 closer to singularity 
        if m < epsilon 
            lambda = (1 - m/epsilon)*5E-2; % smaller the m bigger the damping factor
        else
            lambda = 0;
        end

        invJ = inv(J'*J + lambda *eye(6))*J'; 
        qdot = (invJ*xdot)'; % Finds necessary joint velocities
      
        for j = 1:6 
            if qMatrix(end,j) + deltaT*qdot(j) < robot.model.qlim(j,1) 
                qdot(j) = 0; %stops movement of specific joints if they are about to exceed limits
            elseif qMatrix(end,j) + deltaT*qdot(j) > robot.model.qlim(j,2) 
                qdot(j) = 0; 
            end
        end

        qMatrix = [qMatrix; qMatrix(end,:) + deltaT*qdot]; % Append the new joint configuration to qMatrix
        robot.model.animate(qMatrix(end,:));
        drawnow();
        pause(0.01);
       
    end
end


