
function  draw_mean_square_error(average_projection_error)
    % Function code here
    figure;
    plot(average_projection_error(:,1), average_projection_error(:,2), Color='r')
    xlabel('Number of points')
    ylabel('Avarage Projection Error')
    title('Average Projection Error 3d point with out noise')
    figure;
    plot(average_projection_error(:,1), average_projection_error(:,3))
    xlabel('Number of points')
    ylabel('Avarage Projection Error');
    title('Average Projection Error For Noisy 3d Point')

end
    