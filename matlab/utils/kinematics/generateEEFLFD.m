function eef_model = generateEEFLFD(arm_str, base_pose)
%GENERATEARM Generate arm model
%
%   Usage: arm_model = GENERATEARM(arm_str)
%   @arm_str       dataset string, existing datasets:
%                  'SimpleTwoLinksArm', 'SimpleThreeLinksArm', 'WAMArm', 'PR2Arm'
%   @base_pose     arm's base pose, default is origin with no rotation
%
%   Output Format:
%   arm_model      an ArmModel object, contains kinematics and model information

import gtsam.*
import gpmp2.*

if nargin < 2
    base_pose = Pose3();
end

%  2 link arm 
if strcmp(arm_str, 'SimpleTwoLinksArm')
    % abstract arm
    a = [0.5, 0.5]';
    d = [0, 0]';
    alpha = [0, 0]';
    arm = Arm(2, a, alpha, d);
    % physical arm
    spheres_data = [1  0.0  0.0  0.0  0.01];
    nr_body = size(spheres_data, 1);
    sphere_vec = BodySphereVector;
    for i=1:nr_body
        sphere_vec.push_back(BodySphere(spheres_data(i,1), spheres_data(i,5), ...
            Point3(spheres_data(i,2:4)')));
    end
    eef_model = ArmModel(arm, sphere_vec);
    
% 3 link arm
elseif strcmp(arm_str, 'SimpleThreeLinksArm')
    % abstract arm
    a = [0.5, 0.5, 0.5]';
    d = [0, 0, 0]';
    alpha = [0, 0, 0]';
    arm = Arm(3, a, alpha, d);
    
    % physical arm
    spheres_data = [2  0.0  0.0  0.0  0.01];
    nr_body = size(spheres_data, 1);
    sphere_vec = BodySphereVector;
    for i=1:nr_body
        sphere_vec.push_back(BodySphere(spheres_data(i,1), spheres_data(i,5), ...
            Point3(spheres_data(i,2:4)')));
    end
    eef_model = ArmModel(arm, sphere_vec);

% 7 link WAM arm
elseif strcmp(arm_str, 'WAMArm')
    % arm: WAM arm
    alpha = [-pi/2,pi/2,-pi/2,pi/2,-pi/2,pi/2,0]';
    a = [0,0,0.045,-0.045,0,0,0]';
    d = [0,0,0.55,0,0.3,0,0.06]';
    theta = [0, 0, 0, 0, 0, 0, 0]';
    abs_arm = Arm(7, a, alpha, d, base_pose, theta);
    
    % physical arm
    % sphere data [id x y z r]
    spheres_data = [6 0  0 0 0.01];
    
    nr_body = size(spheres_data, 1);
    
    sphere_vec = BodySphereVector;
    for i=1:nr_body
        sphere_vec.push_back(BodySphere(spheres_data(i,1), spheres_data(i,5), ...
            Point3(spheres_data(i,2:4)')));
    end
    eef_model = ArmModel(abs_arm, sphere_vec);

% 7 DOF PR2 right arm
elseif strcmp(arm_str, 'PR2Arm')
    % arm: PR2 arm
    alpha = [-1.5708, 1.5708, -1.5708, 1.5708, -1.5708, 1.5708, 0]';
    a = [0.1, 0, 0, 0, 0, 0, 0]';
    d = [0, 0, 0.4, 0, 0.321, 0, 0]';
    theta = [0, 1.5708, 0, 0, 0, 0, 0]';
    abs_arm = Arm(7, a, alpha, d, base_pose, theta);
    % physical arm
    % sphere data [id x y z r]
    spheres_data = [6 0 0 0 0.01];

    nr_body = size(spheres_data, 1);
    
    sphere_vec = BodySphereVector;
    for i=1:nr_body
        sphere_vec.push_back(BodySphere(spheres_data(i,1), spheres_data(i,5), ...
            Point3(spheres_data(i,2:4)')));
    end
    eef_model = ArmModel(abs_arm, sphere_vec);

% 6 DOF JACO2 arm
elseif strcmp(arm_str, 'JACO2Arm')
    % arm: JACO2 6DOF arm
    alpha = [pi/2, pi, pi/2, 1.0472, 1.0472, pi]';
    a = [0, 0.41, 0, 0, 0, 0]';
    d = [0.2755, 0, -0.0098, -0.2501, -0.0856, -0.2228]';
    theta = [0, -pi/2, pi/2, 0, -pi, pi/2]';
    abs_arm = Arm(6, a, alpha, d, base_pose, theta);
    % physical arm
    % sphere data [id x y z r]
    spheres_data = [5 0.0 0.0 0.0 0.01];

    nr_body = size(spheres_data, 1);
    
    sphere_vec = BodySphereVector;
    for i=1:nr_body
        sphere_vec.push_back(BodySphere(spheres_data(i,1), spheres_data(i,5), ...
            Point3(spheres_data(i,2:4)')));
    end
    eef_model = ArmModel(abs_arm, sphere_vec);

% 7 DOF JACO2 arm
elseif strcmp(arm_str, 'JACO2_7')
    % arm: JACO2 7DOF arm
    alpha = [-pi/2, pi/2, pi/2, pi/2, pi/2, pi/2, pi]';
    a = [0, 0, 0, 0, 0, 0, 0]';
    d = [0.2755, 0, -0.41, -0.0098, -0.3111, 0, -0.2638]';
    theta = [pi, 0, 0, 0, 0, 0, pi/2]';
    abs_arm = Arm(7, a, alpha, d, base_pose, theta);
    % physical arm
    % sphere data [id x y z r]
    spheres_data = [6 0.0 0.0 0.0 0.01];

    nr_body = size(spheres_data, 1);
    
    sphere_vec = BodySphereVector;
    for i=1:nr_body
        sphere_vec.push_back(BodySphere(spheres_data(i,1), spheres_data(i,5), ...
            Point3(spheres_data(i,2:4)')));
    end
    eef_model = ArmModel(abs_arm, sphere_vec);
    
    % no such dataset
else
    error('No such arm exist');
end

end
