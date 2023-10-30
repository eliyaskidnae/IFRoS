
 au = 557.0943 ; av = 712.9824;
 u0 = 326.3819 ; v0 = 298.6679 ;
    % location of the world referance 
    Tx = 100 ; Ty= 0 ; Tz = 1500 ;
    Phix = 0.8*pi/2;
    Phiy = -1.8*pi/2;
    Phix1 = pi/5;
    
    K = [au , 0, u0;0,av,v0; 0,0,1];
    % Rotation in x-->roty--> rotx
    Rot_x  = [1,0,0;0,cos(Phix),-sin(Phix);0,sin(Phix),cos(Phix)];
    Rot_y = [cos(Phiy),0,sin(Phiy); 0,1,0; -sin(Phiy), 0, cos(-Phiy)];
    Rot_x1  = [1,0,0;0,cos(Phix1),-sin(Phix1);0,sin(Phix1),cos(Phix1)];
    Rot_mat = Rot_x*Rot_y*Rot_x1;
    Rot = [Rot_mat;0,0,0];
    Tran = [100;0;1500;1];
    
    cRw = [ Rot , Tran]; % transformation Matrices

    P= K*[1,0,0,0;0,1,0,0;0,0,1,0]*cRw ;  % Step 2-Compute Projection Matrix Initial 
                                           %  from intial from intinisic and extrinisic parametrs
   
    % p2d= calculate_2d_points(6,p3d,P)
    % scatter(p2d(:,1) , p2d(:,2) )
    % xlabel('X-axis')
    % ylabel('Y-axis')
    % title('2d points Scatter Plot')

    % Working for 6 points 
    % min_range = -480 ;% min range to genarate for 3d points
    % max_range = 480 ; % max Range 
    % num_points = 6 
    % p3d = getRandom(min_range,max_range , num_points);
    % 
    % 2d Point    
    % p2d = calculate_2d_points(num_points,p3d,P) % get p2d point by projecting in p from the 3d points
    % P_Estimated = get_estimated_p(p2d,p3d,num_points,P)
    % % Add noise to 2d Data 
    % 
    % [K_new,cRw_new1] = get_intrinsics_from_proj_matrix(P_Estimated);
    % noise = getGaussianNoise(-1,1 , num_points);
    % 
    % p2d_noise = p2d + noise;
    % P_Estimated_noise = get_estimated_p(p2d_noise,p3d,num_points,P);
    % 
    % %Step 9 compute 2dpoints 
    % p2d_es1 = calculate_2d_points(num_points,p3d,P_Estimated)
    % error1_point6 = calculate_mean_square_diff(p2d_es1,p2d)
    % 
    % p2d_es_noise = calculate_2d_points(num_points,p3d,P_Estimated_noise)
    % error2_point6= calculate_mean_square_diff(p2d_es_noise,p2d)
    % 
    % 
    % % Step 10 Increaze the number of points
    % % 10 -points
    % num_points2 =10 
    % p3d_point10 = getRandom(min_range,max_range , num_points2);
    % p2d_points10 = get_2dPoints(num_points2,p3d_point10,P) % get p2d point by projecting in p from the 3d points
    % P_estim_10 = get_estimated_p(p2d_points10,p3d_point10,num_points,P);
    % noise = getGaussianNoise(-1,1 , num_points);
    % % 
    % p2d_noise_point10 = p2d_points10 + noise; % add noise to my 2d points
    % P_estim_noise_p10 = get_estimated_p(p2d_noise_point10,p3d_point10,num_points2,P);
    % % 
    % % %Step 9 compute 2dpoints 
    % p2d_es_point10 = calculate_2d_points(num_points2,p3d_point10,P_estim_noise_p10)
    % error1_point10 = calculate_mean_square_diff(p2d_es_point10,p2d_points10)
    % % 
    % p2d_es_noise = calculate_2d_points(num_points2,p3d_point10,P_estim_noise_p10)
    % error2_point10= calculate_mean_square_diff(p2d_noise_point10,p2d_points10)
    % Add noise to 2d Data 
    num_points = 6:6; % number of points to estimate
    average_projection_error= [];
    i=1;
    while i<=length(num_points);
        num_point = num_points(i);     
        [error1,error2]= projection_handler(num_point,P);
        average_projection_error= [average_projection_error;num_point, error1,error2];
        i=i+1;
    end
    % figure;
    % plot(average_projection_error(:,1), average_projection_error(:,2))
    % xlabel('Number of points')
    % ylabel('Avarage Projection Error')
    % 
    % plot(average_projection_error(:,1), average_projection_error(:,3))
    % xlabel('Number of points')
    % ylabel('Avarage Projection Error');

function [error1,error2] = projection_handler(num_points,P)
    min_range = -480 ;% min range to genarate for 3d points
    max_range = 480 ; % max Range 
 
    p3d= get_random_3d_points(min_range,max_range , num_points);
    p2d= calculate_2d_points(num_points,p3d,P); % get p2d point by projecting in p from the 3d points
    estimated_P = get_estimated_p(p2d,p3d,num_points,P);

    [K_new1,cRw_new1] = get_intrinsics_from_proj_matrix(P);
    
    K_new1

    noise = getGaussianNoise(-1,1 ,num_points);
   
    p2d_noise = p2d + noise; % add noise to the point 2d
    noise_estimated_P = get_estimated_p(p2d_noise,p3d,num_points,P);
    % 
    % %Step 9 compute 2dpoints 
    p2d_estimated= calculate_2d_points(num_points,p3d,estimated_P);
    error1 = calculate_avarege_projection_error(p2d_estimated,p2d);
    
    % 
    p2d_estimated_noise = calculate_2d_points(num_points,p3d,noise_estimated_P);
    error2 = calculate_avarege_projection_error(p2d_estimated_noise,p2d);
    

end
function random_num = get_random_3d_points(min_range, max_range , num_pooints);
    random_num = min_range + (max_range - min_range)*rand(num_pooints,3);
end

function [K,cRw] = get_intrinsics_from_proj_matrix(P)   
     % Get left-side 3x3 block of P
     M = P(:,1:3);   
     % This implements the RQ decomposition from QR in matlab 
     [Q,R] = qr(rot90(M,3));  
     R = rot90(R,2)';  Q = rot90(Q);   
     % Check the determinant of Q to make cRw a proper rotation 
     if det(Q) < 0 ;
       cRw = -Q; 
     else     
       cRw = Q; end   % Get the normalized intrinsics
     K = R/R(3,3); 
 end
 

 function noise = getGaussianNoise(lower_limit,upper_limit,num_points)
    % Define the range and distribution parameters
    mu = 0;            % Mean of the Gaussian distribution
    sigma = 0.51;        % Standard deviation of the Gaussian distribution

    % Generate random numbers from the Gaussian distribution b/n [-1,1]
    % affecting 95% of the point area 
    noise = normrnd(mu, sigma, [num_points,2]); % Adjust the number of samples (1000 in this example)

    % Scale and shift the generated numbers to fit within the desired range
    % scaled_and_shifted_noise = (noise - mean(noise)) * (upper_limit - lower_limit) / std(noise) + (upper_limit + lower_limit) / 2;

 end


 function P_Estimated = get_estimated_p(p2d , p3d,num_points,P)
    % initializing 
    Q=zeros(12,11);
    B2= zeros(12,1);
    % callculate the projection matrices using method of hall 
    for i=1:num_points;
        p3d(i,:);
        
        Q(2*i-1,:) = [p3d(i,:),1,0,0,0,0,-p2d(i,1)*p3d(i,1), -p2d(i,1)*p3d(i,2),-p2d(i,1)*p3d(i,3)];
        Q(2*i,:) = [0,0,0,0,p3d(i,:),1,-p2d(i,2)*p3d(i,1), -p2d(i,2)*p3d(i,2),-p2d(i,2)*p3d(i,3) ];
        B(2*i-1,1) = p2d(i,1);
        B(2*i,1) = p2d(i,2);
    end 

    A = inv(Q'*Q)*Q'*B; % Getting the unkown variables in colomun vector

    A = [A;1];

    P_Estimated = reshape(A,4,3)'*P(3,4); % getting the estimated P by reshaping the calculated unkown variables 
                                         % and multiplyin by the P(3,4) to
                                         % make 
                                         % 

 end

 function p2d = calculate_2d_points(num_points,p3d,P)
    % claculate the 2d points in the image plan from the camera projection
    % matrices and the 3d points 
    p2d_homeg = [];
    for i=1: num_points      
        point = P*[p3d(i,:),1]';
        p2d_homeg = [p2d_homeg ; point'];

    end
    %Change to Cartishian     %
    p2d = p2d_homeg(:, 1:2) ./ p2d_homeg(:, 3); 
 end

 function error = calculate_avarege_projection_error(matrix1,matrix2)
    % Calculate the element-wise squared differences
    squared_differences = (matrix1 - matrix2).^2;
    distance = sum(squared_differences,2);
    error= mean(distance);
    
 end