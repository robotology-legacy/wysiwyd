
%% author: Marius Leordeanu
% last updated: Feb 25, 2011

% for questions contact the author at: leordeanu@gmail.com


%% please cite the following paper:
%  
% An Integer Projected Fixed Point Method for Graph Matching and MAP
% Inference, NIPS 2009
% by Marius Leordeanu ,  Martial Hebert ,  Rahul Sukthankar 

% Utility:
% this function tries to maximize the matching score x'Mx 
% where x obeys discrete one-to-one matching constraints 
% such that x(i) = 1 if feature nodes(i) from one image is matched to feature labels(i) from the other
% and 0 otherwise


% Note: sol0 is the initial solution


function [sol, stats]  = ipfp_gm(M, L12)

%%  scores, dX, best_score, discreteRate, stepSize_t, stepSize_norm]

labels = L12(:,1)';
nodes = L12(:,2)';
sol0 = ones(length(M),1)/length(M);

tic;

maxSteps = 50;

n = length(labels);

nLabels = max(labels);
nNodes = max(nodes);

new_sol = sol0;
best_sol = sol0;


best_score = 0; 

nSteps = 0;

scores(1) = new_sol'*M*new_sol;
   
scores2(1) = new_sol'*M*new_sol;

discreteRate = 0;

converged = 0;

while 1
       
   if nSteps > maxSteps
       break
   end
         
   nSteps =  nSteps + 1; 
   
   old_sol = new_sol;
      
   xx =  M*old_sol;

   A = -Inf*ones(nNodes, nLabels);

   for i = 1:nNodes

       f = find(nodes == i);
       
       A(i, labels(f)) = xx(f);

   end
   
   A = max(A(:))-A;
   
   [X, score]=hungarian(A);
   
   x2 =  zeros(n,1);
   
   for i = 1:nNodes
      
      f = find(nodes == i);
      
      match_ind = find(X(i,labels(f)) == 1);
      
      x2(f(match_ind)) = 1;
     
   end
       
   k = (x2 - old_sol)'*M*(x2 - old_sol);
  
   t = 1;
   
   if k >= 0
   
       new_sol = x2;
   
       stepSize_t(nSteps) = 1;
             
       stepSize_norm(nSteps) = norm(x2 - old_sol);
        
       if converged == 0
           discreteRate = discreteRate + 1;
       end
           
   else
 
       c = old_sol'*M*(x2-old_sol);
       
       t = min([1, -c/k]);
       
       if t < 0.01
           t = 0;
       end
       
       if t == 1 && converged == 0
          discreteRate = discreteRate + 1;
       end
       
       new_sol = old_sol + t*(x2 - old_sol);
       
       stepSize_t(nSteps) = -c/k;
       
       stepSize_norm(nSteps) = norm(x2 - old_sol);
     
   end
      
   scores(nSteps+1) = new_sol'*M*new_sol;
   
   scores2(nSteps+1) = new_sol'*M*old_sol;
   
   dX(nSteps) = sum(abs(new_sol - best_sol));
     
   curr_score = x2'*M*x2;
   
   if curr_score > best_score
       
        best_score = curr_score; 
        
        best_sol = x2;
       
   end
      
   if norm(new_sol - old_sol) == 0
       break;
   end
   
end


discreteRate = discreteRate/nSteps;

sol = best_sol;

stats.dX = dX;
stats.scores = scores;
stats.scores2 = scores2;
stats.best_score = best_score;

stats.discreteRate = discreteRate;

stats.stepSize_t = stepSize_t;
stats.stepSize_norm = stepSize_norm;

% toc;

return