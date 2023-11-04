
 % STEP1: Intialize the intrinsic parameters and extrinsic parameters
 
 %define intrinisic parametrs
 au = 557.0943 ; av = 712.9824; % 
 u0 = 326.3819 ; v0 = 298.6679 ; % 
 % Location of the world reference frame in camera coordinates in mm %
 Tx = 100 ; Ty= 0 ; Tz = 1500 ;
 Phix = 0.8*pi/2;
 Phiy = -1.8*pi/2;
 Phix1 = pi/5;
 
 %Step 2: Create the camera intrinsic matrix K and 
 % the extrinsic camera transformation cTw from the values defined above. 

 K = [au , 0, u0 ; 0, av, v0 ; 0, 0, 1]; %intrinsic matrix K

 % World rotation w.r.t. camera coordinates % Euler XYX1 angles 
 % Rotation in x-axis by Phix degree
 Rot_x =[1,0,0;0,cos(Phix),-sin(Phix);0,sin(Phix),cos(Phix)]; 
 %Rotation in y-axis by Phiy degree
 Rot_y = [cos(Phiy),0,sin(Phiy); 0,1,0; -sin(Phiy), 0, cos(-Phiy)]; 
 %Roatation in z-axis by Phix1 de
 Rot_x1 = [1,0,0;0,cos(Phix1),-sin(Phix1);0,sin(Phix1),cos(Phix1)]; 
 Rot_mat = Rot_x*Rot_y*Rot_x1; % combining the rotation matrixes by mult
 Rot = [Rot_mat;0,0,0]; % changing to homogeneous coordinate

 % Tx = 100; Ty = 0; Tz = 1500;  
 Tran = [100;0;1500;1]; % Translation matrix
    
 cTw = [ Rot , Tran]; % World to Camera Transformation Matrices

 P= K*[1,0,0,0;0,1,0,0;0,0,1,0]*cTw ;  % Calaculating Projection Matrix  
                                       % from intinisic and extrinisic parametrs
 % STEP3: Define 6 random points
 %Working for 6 points 
 min_range = -480 ;% min range to genarate for 3d points
 max_range = 480 ; % max Range 
 num_points = 6 
 p3d = get_random_3d_points(min_range,max_range , num_points);
  
 % STEP 4 : projection of the 3D points on the image plane
 %     by using the camera projection matrix

 p2d= calculate_2d_points(6,p3d,P) % calculate the 2d points from 3d points using P

 % STEP5 : Plot the 2D points
 scatter(p2d(:,1) , p2d(:,2) , 50 , 'b' , 'o', 'filled') % draw the graph 
 xlabel('X-axis')
 ylabel('Y-axis')
 title('2d points Scatter Plot')

 num_points = 6:100; % number of points to estimate
 average_projection_error= [];
 i=1;
 while i<=length(num_points)
     num_point = num_points(i);     
     [error1,noise_error]= projection_handler(num_point,P,K);
     average_projection_error = [average_projection_error;num_point, error1,noise_error];
     i=i+1;
 end

 draw_mean_square_error(average_projection_error)


 % % res = plot_function(average_projection_error)

 function [error1,noise_error] = projection_handler(num_points,P,K)
    min_range = -480 ;% min range to genarate for 3d points
    max_range = 480 ; % max Range 
 
    p3d= get_random_3d_points(min_range,max_range , num_points);
    p2d= calculate_2d_points(num_points,p3d,P); % get p2d point by projecting in p from the 3d points
    estimated_P = get_estimated_projecion_matrix(p2d,p3d,num_points,P);

    [K_new,cRw_new] = get_intrinsics_from_proj_matrix(estimated_P);
    
    noise = getGaussianNoise(-1,1 ,num_points);
  
    p2d_noise = p2d + noise; % add noise to the point 
    P_noisy = get_estimated_projecion_matrix(p2d_noise , p3d ,num_points ,P);
    noisy_estimated_P = get_estimated_projecion_matrix(p2d_noise,p3d,num_points,P);

    [K_noise,cRw_noise] = get_intrinsics_from_proj_matrix(noisy_estimated_P);


    P_difference = P - estimated_P;
    P_noise_difference =P- P_noisy;
    K_difference = K - K_new;
    K_noise_difference = K - K_noise;
    % 
    % %Step 9 compute 2dpoints 
    p2d_estimated= calculate_2d_points(num_points,p3d,estimated_P);
    error1 = calculate_avarege_projection_error(p2d_estimated,p2d);
    
    % 
    p2d_estimated_noise = calculate_2d_points(num_points,p3d,noisy_estimated_P);
    noise_error = calculate_avarege_projection_error(p2d_estimated_noise,p2d);
 
 end

function random_num = get_random_3d_points(min_range, max_range , num_pooints);
    random_num = min_range + (max_range - min_range)*rand(num_pooints,3);
end

function p2d = calculate_2d_points(num_points,p3d,P)
    % claculate the 2d points in the image plan from the camera projection
    % matrices and the 3d points 
    p2d_homeg = [];
    for i=1: num_points      
        point = P*[p3d(i,:),1]';
        p2d_homeg = [p2d_homeg ; point'];

    end
    %Change to Cartishian%
    p2d = p2d_homeg(:, 1:2) ./ p2d_homeg(:, 3); 
 end


function noise = getGaussianNoise( lower_limit, upper_limit,num_points)
    % Define the range and distribution parameters
    mu = 0;            % Mean of the Gaussian distribution
    sigma = 0.51;        % Standard deviation of the Gaussian distribution

    % Generate random numbers from the Gaussian distribution b/n [-1,1]
    % affecting 95% of the point area 
    noise = normrnd(mu, sigma, num_points,2); % Adjust the number of samples
    noisy_indices = randperm(num_points, round(0.05*num_points));%applaying noise only to 95% of the points 
    noise(noisy_indices,:)=0; % no noise to 5% of the 2d points
    
 end

 function error = calculate_avarege_projection_error(p2d_new,p2d)
    % Calculate the element-wise squared differences
    squared_differences = (p2d_new - p2d).^2;
    distance = sum(squared_differences,2);
    error= mean(distance);
 end