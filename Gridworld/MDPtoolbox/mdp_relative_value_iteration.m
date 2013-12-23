function [U, policy, g, cpu_time] = mdp_relative_value_iteration(P, R, epsilon, max_iter)


% mdp_relative_value_iteration   Resolution of MDP with average reward
%                                with relative value iteration algorithm 
% Arguments -------------------------------------------------------------
% Let S = number of states, A = number of actions
%   P(SxSxA) = transition matrix 
%              P could be an array with 3 dimensions or 
%              a cell array (1xA), each cell containing a matrix (SxS) possibly sparse
%   R(SxSxA) or (SxA) = reward matrix
%              R could be an array with 3 dimensions (SxSxA) or 
%              a cell array (1xA), each cell containing a sparse matrix (SxS) or
%              a 2D array(SxA) possibly sparse  
%   epsilon  = epsilon-optimal policy search, upper than 0, 
%              optional (default: 0.01)
%   max_iter = maximum number of iteration to be done, upper than 0,
%              optional (default 1000)
% Evaluation -------------------------------------------------------------
%   U(S)     =  value function
%   policy(S) = epsilon-optimal policy
%   g        = gain of the policy
%   cpu_time = used CPU time
%--------------------------------------------------------------------------
% In verbose mode, at each iteration, displays the span of U variation
% and the condition which stopped iterations : epsilon-optimum policy found
% or maximum number of iterations reached.

% MDPtoolbox: Markov Decision Processes Toolbox
% Copyright (C) 2009  INRA
% Redistribution and use in source and binary forms, with or without modification, 
% are permitted provided that the following conditions are met:
%    * Redistributions of source code must retain the above copyright notice, 
%      this list of conditions and the following disclaimer.
%    * Redistributions in binary form must reproduce the above copyright notice, 
%      this list of conditions and the following disclaimer in the documentation 
%      and/or other materials provided with the distribution.
%    * Neither the name of the <ORGANIZATION> nor the names of its contributors 
%      may be used to endorse or promote products derived from this software 
%      without specific prior written permission.
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
% IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
% INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
% BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
% DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
% LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
% OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
% OF THE POSSIBILITY OF SUCH DAMAGE.


cpu_time = cputime;

global mdp_VERBOSE;
if ~exist('mdp_VERBOSE'); mdp_VERBOSE=0; end;

% check of arguments
if nargin > 3 & epsilon <= 0 
    disp('--------------------------------------------------------')
    disp('MDP Toolbox ERROR: epsilon must be upper than 0')
    disp('--------------------------------------------------------')
elseif nargin > 4 & max_iter <= 0
    disp('--------------------------------------------------------')
    disp('MDP Toolbox ERROR: The maximum number of iteration must be upper than 0')
    disp('--------------------------------------------------------')
else
    
    if iscell(P)
        S = size(P{1},1);
        A = length(P);
    else
        S = size(P,1);
        A = size(P,3);
    end
    
    PR = mdp_computePR(P,R);

    % set default values
    if nargin < 4; max_iter = 1000; end;
    if nargin < 3; epsilon = 0.01; end;
     
    U=zeros(S,1);

    if mdp_VERBOSE; disp('  Iteration  U_variation'); end;

    iter = 0;
    is_done = false;
    while ~is_done
         
        iter = iter + 1;
        
        if iscell(P)
            for a=1:A; Q1(a)=PR(S,a) + P{a}(S,:)*U; end;
        else
            for a=1:A; Q1(a)=PR(S,a) + P(S,:,a)*U; end;
        end
        
        g=max(Q1);
        
        if iscell(P)
            for a=1:A; Q2(:,a) = PR(:,a) + P{a}* U; end;
        else
            for a=1:A; Q2(:,a) = PR(:,a) + P(:,:,a)* U; end;
        end
        
        [Unext,policy]=max(Q2,[],2);
        Unext = Unext - g;
        
        variation = mdp_span(Unext-U);
        if mdp_VERBOSE 
             disp(['      ' num2str(iter,'%5i') '         ' num2str(variation)]); 
        end;

        if variation < epsilon
             is_done = true; 
             if mdp_VERBOSE 
                 disp('MDP Toolbox : iterations stopped, epsilon-optimal policy found')
             end;
        elseif iter == max_iter
             is_done = true; 
             if mdp_VERBOSE 
                 disp('MDP Toolbox : iterations stopped by maximum number of iteration condition')
              end;
       else
             U = Unext; 
        end
     end;
 
end;

cpu_time = cputime - cpu_time;
