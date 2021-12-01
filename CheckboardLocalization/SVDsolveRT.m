function [R, T, err] = SVDsolveRT(P, Q)
    % «ÛR∫ÕT
    P_center = P - mean(P,2);
    Q_center = Q - mean(Q,2);
    [U,S,V] = svd(Q_center*P_center');
    R = V*U';
    
    if abs(det(R)+1)<0.00001
        R = V * diag([1,1,-1]) * U';
    end
    
    T = mean(P,2) - R * mean(Q,2);
    
    %reprojection error
    Q_proj = R * Q + T;
    err = mean((Q_proj(1,:)-P(1,:)).^2 + (Q_proj(2,:)-P(2,:)).^2 + (Q_proj(3,:)-P(3,:)).^2);
end