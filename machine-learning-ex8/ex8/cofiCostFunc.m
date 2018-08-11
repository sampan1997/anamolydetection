function [J, grad] = cofiCostFunc(params, Y, R, num_users, num_movies, ...
                                  num_features, lambda)
%COFICOSTFUNC Collaborative filtering cost function
%   [J, grad] = COFICOSTFUNC(params, Y, R, num_users, num_movies, ...
%   num_features, lambda) returns the cost and gradient for the
%   collaborative filtering problem.
%

% Unfold the U and W matrices from params
X = reshape(params(1:num_movies*num_features), num_movies, num_features);
Theta = reshape(params(num_movies*num_features+1:end), ...
                num_users, num_features);

            
% You need to return the following values correctly
J = 0;
X_grad = zeros(size(X));
Theta_grad = zeros(size(Theta));


% ====================== YOUR CODE HERE ======================
% Instructions: Compute the cost function and gradient for collaborative
%               filtering. Concretely, you should first implement the cost
%               function (without regularization) and make sure it is
%               matches our costs. After that, you should implement the 
%               gradient and use the checkCostFunction routine to check
%               that the gradient is correct. Finally, you should implement
%               regularization.
%
% Notes: X - num_movies  x num_features matrix of movie features 
%        Theta - num_users  x num_features matrix of user features
%        Y - num_movies x num_users matrix of user ratings of movies
%        R - num_movies x num_users matrix, where R(i, j) = 1 if the 
%            i-th movie was rated by the j-th user
%
% You should set the following variables correctly:
%
%        X_grad - num_movies x num_features matrix, containing the 
%                 partial derivatives w.r.t. to each element of X
%        Theta_grad - num_users x num_features matrix, containing the 
%                     partial derivatives w.r.t. to each element of Theta
%
R=R';%NO OF USERS*NO OF MOVIES
Y=Y';%(no of users*no of movies)
for i=1:num_users..................................................%cost function calculation start
for j=1:num_movies
    if(R(i,j))
      J=J+(((Theta(i,:)*(X(j,:))')-Y(i,j))^2);
    end
  end
  end
 
for i=1:num_users
  for j=1:num_features
    J=J+(lambda*Theta(i,j)*Theta(i,j));
  end
end
for i=1:num_movies
  for j=1:num_features
    J=J+(lambda*X(i,j)*X(i,j));
  end
end
J=J/2;................................................................%cost function calculation ends
  
  
.................................................%gradient calculation start w.r.t theta parameters  
t=0;
for i=1:num_users
  %for 1st user
for k=1:num_features
  %1ST FEATURE
for j=1:num_movies
   if(R(i,j))
    t=t+((Theta(i,:)*(X(j,:))')-Y(i,j))*X(j,k);
    %p=p+((Theta(i,:)*X(j,:))-Y(i,j))*X(i,k);
   end
 end
 Theta_grad(i,k)=t;
 t=0;
end
end
for i=1:num_users
for j=1:num_features
  Theta_grad(i,j)=Theta_grad(i,j)+lambda*Theta(i,j);
end
end

%...................................end of gradient theta without regularization
%...................................start of gradient X without regularization
R=R';%no olf movies*nom of users
Y=Y';%.........,,..............
p=0;
for i=1:num_movies
  %num_movies
  idx=find(R(i,:)==1)
%for k=1:num_features
  Theta_temp=Theta(idx,:)%.......nu*nf
  X_temp=X(i,:);%.......1*nf
  Y_temp=Y(i,idx);.........1*nu
 %for j=1:num_users
   % if(R(i,j))
   % t=t+((Theta(i,:)*X(j,:))-Y(i,j))*Theta(i,k);
   X_grad(i,:)=((X_temp*Theta_temp'-Y_temp)*Theta_temp)+(lambda*X_grad(i,:));%1*nf
    %p=p+((Theta(j,:)*(X(i,:))')-Y(i,j))*Theta(j,k);
end
 
%end
%X_grad(i,k)=p;
%p=0;
%end
%end
for i=1:num_movies
   for j=1:num_features
     X_grad(i,j)=X_grad(i,j)+lambda*X(i,j);
   end
end
 

%=============================================================

grad = [X_grad(:); Theta_grad(:)];

end
