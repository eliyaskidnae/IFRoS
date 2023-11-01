


function P_Estimated = get_estimated_projecion_matrix(p2d , p3d,num_points,P)
    % initializing 
    Q=zeros(12,11);
    B2= zeros(12,1);
    % callculate the projection matrices using method of hall 
    for i=1:num_points;

        Q(2*i-1,:) = [p3d(i,:),1,0,0,0,0,-p2d(i,1)*p3d(i,1), -p2d(i,1)*p3d(i,2),-p2d(i,1)*p3d(i,3)];
        Q(2*i,:) = [0,0,0,0,p3d(i,:),1,-p2d(i,2)*p3d(i,1), -p2d(i,2)*p3d(i,2),-p2d(i,2)*p3d(i,3) ];
        B(2*i-1,1) = p2d(i,1);
        B(2*i,1) = p2d(i,2);

    end 

    A = inv(Q'*Q)*Q'*B; % Getting the unkown variables in colomun vector

    A = [A;1];

    P_Estimated = reshape(A,4,3)'*P(3,4); % getting the estimated P by reshaping the calculated unkown variables 
                                          % and multiplyin by the P(3,4) to
                                        

 end