function SET = incSVDD(xin,yin,C,kernel_type,kernel_param,subsequence,SET)

[ndata_in,ndim_in] = size(xin);

if nargin == 1
    yin = ones(ndata_in,1);
    C = 1;
    kernel_param = 1;
    kernel_type = 'gaussian';

elseif nargin == 2
    C = 1;
    kernel_param = 1;
    kernel_type = 'gaussian';

elseif nargin < 7
    SET = SETinit;
end

num_already_trained = SET.S.ndata + SET.E.ndata + SET.O.ndata;

%------------------------------- init start -----------------------------------
if C > 1
    C = 1;
end

if subsequence == 0 | num_already_trained == 0  % 새롭게 학습하는 경우
    SET = SETinit;

    x = [];
    y = [];
    alpha = [];
    g = [];

    Sidx = [];
    Eidx = [];
    Oidx = [];
    cidx = [];

elseif subsequence == 1 % 기존에 학습한 결과에 이어서 학습하는 경우

    x = [SET.S.x ; SET.E.x ; SET.O.x];
    y = [SET.S.y ; SET.E.y ; SET.O.y];
    alpha = [SET.S.alpha ; SET.E.alpha ; SET.O.alpha];
    g = [SET.S.g ; SET.E.g ; SET.O.g];
    SV = SET.S.x;

    Sidx = [1:SET.S.ndata]';
    Eidx = [SET.S.ndata+1:SET.S.ndata+SET.E.ndata]';
    Oidx = [SET.S.ndata+SET.E.ndata+1:SET.S.ndata+SET.E.ndata+SET.O.ndata]';
    Qxx = [];
end

ndata_condition = floor(1/C);

%------------------------------- init end ------------------------------------


for in_data_num = 1:ndata_in
    xc = xin(in_data_num,:);
    yc = yin(in_data_num);

    if subsequence == 0
        ndata = in_data_num;
    else
        ndata = num_already_trained+1;
    end


    %%%%%%%%%%%%%%%%%%%%%%%% 초기 조건 만족 못했을 경우 %%%%%%%%%%%%%%%%%%%%%%
    %   Take the first floor(1/C) data from target class,
    %   set their Lagrange multipliers ‘C’ and put them to the set E.
    %
    
    if ndata <= ndata_condition
        x = [x;xc];
        y = [y;yc];

        alpha = [alpha;C];
        Eidx = [Eidx;in_data_num];
        Sidx = [];
        Oidx = [];

        Mij = genQmtx(x,y,kernel_type,kernel_param) * alpha;
        g = -y + 2*Mij;

        SET = SETup(x,y,alpha,g,Sidx,Eidx,Oidx);
        %%%%%%%%%%%%%%%%%%%%%%%% 초기 조건 만족 했을 경우 %%%%%%%%%%%%%%%%%%%%%%%%
    else
        if ndata == ndata_condition+1
            x = [x;xc];
            y = [y;yc];

            alphac = 1-floor(1/C)*C; % Take the next object c, assign ac = 1-floor(1/C)*C
            alpha = [alpha;alphac];

            Mij = genQmtx(x,y,kernel_type,kernel_param) * alpha;
            g = -y + 2*Mij;
            mu = -max(g(1:end-1))-1e-12;
            g = g + mu*y;

            Qxx = genQmtx(x,y,kernel_type,kernel_param);
            cidx = size(x,1);

            Q = inf;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%% 만족 후 학습 시작 %%%%%%%%%%%%%%%%%%%%%%%%%%%
        if ndata > ndata_condition+1
            if isempty(Qxx)   % Qxx & Q 가 만들어져 있지 않은 경우 기존의 학습 결과를 이용하여 새로 만듦
                disp('Calculate Qxx......');
                x = [SET.S.x ; SET.E.x ; SET.O.x];
                y = [SET.S.y ; SET.E.y ; SET.O.y];
                alpha = [SET.S.alpha ; SET.E.alpha ; SET.O.alpha];
                g = [SET.S.g ; SET.E.g ; SET.O.g];

                Sidx = [1:SET.S.ndata]';
                Eidx = [SET.S.ndata+1:SET.S.ndata+SET.E.ndata]';
                Oidx = [SET.S.ndata+SET.E.ndata+1:SET.S.ndata+SET.E.ndata+SET.O.ndata]';

                Qxx = genQmtx(x,y,kernel_type,kernel_param);

                P = [0, y(Sidx)'; y(Sidx), 2*Qxx(Sidx,Sidx)];
                Q = inv(P);
            end

            % 새로 들어온 xc에 대한 Qxc 계산
            x = [x;xc];
            y = [y;yc];

            Qxc = genQc(x,y,xc,yc,kernel_type,kernel_param);
            Qxx = [Qxx,Qxc(1:end-1);Qxc(1:end-1)',1];

            ndata = size(x,1);
            cidx = ndata;

            alphac = 0;
            alpha = [alpha;alphac];

            % compute gc
            if isempty(Sidx)
                gc = -1*y(cidx) + 2*sum(alpha.*Qxc,1) + mu*y(cidx);
            else
                [R2, mu] = boundary(x,y,alpha,Sidx(1));
                gc = -1*y(cidx) + 2*sum(alpha.*Qxc,1) + mu*y(cidx);
            end
            g = [g;gc];
        end
      
        iter = 1;

        % C가 inlier인 경우
        if g(cidx) > 0   % c -> O
            iter = 0;

            Oidx = [Oidx;cidx];
            %%%%%%%%%%%%%%%%%%%%%% 결과 plot %%%%%%%%%%%%%%%%%%%%%%
            % incSVDD_drawing(time_delay);
            % disp('c to O');
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end

        while(iter)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Compute beta and gamma
            beta = zeros(ndata,1);
            gamma = zeros(ndata,1);

            if isempty(Sidx)
                gamma = y(cidx)*y([1;Eidx;Oidx]);
                beta(cidx) = -inf;
            else
                %------------------ Computing beta --------------------
                beta_buf = -Q * [y(cidx) ; 2*Qxx(cidx,Sidx)'];

                beta_buf_Sidx = 1 + [1:size(Sidx,1)];
                beta(Sidx) = beta_buf(beta_buf_Sidx);

                beta(cidx) = beta_buf(1);

                %------------------ Computing gamma -------------------                
                if isempty(Eidx) & isempty(Oidx)
                    gamma_buf = [y(cidx), 2*Qxx(cidx,Sidx)] * beta_buf...
                        + 2*[Qxx(cidx,cidx)];                    
                elseif isempty(Eidx) & ~isempty(Oidx)
                    gamma_buf = [y(cidx), 2*Qxx(cidx,Sidx) ; y(Oidx), 2*Qxx(Oidx,Sidx)] * beta_buf...
                        + 2*[Qxx(cidx,cidx) ; Qxx(cidx,Oidx)'];
                elseif ~isempty(Eidx) & isempty(Oidx)
                    gamma_buf = [y(cidx), 2*Qxx(cidx,Sidx) ; y(Eidx), 2*Qxx(Eidx,Sidx)] * beta_buf...
                        + 2*[Qxx(cidx,cidx) ; Qxx(cidx,Eidx)'];
                else
                    gamma_buf = [y(cidx), 2*Qxx(cidx,Sidx) ; y(Eidx), 2*Qxx(Eidx,Sidx) ; y(Oidx), 2*Qxx(Oidx,Sidx)] * beta_buf...
                        + 2*[Qxx(cidx,cidx) ; Qxx(cidx,Eidx)'; Qxx(cidx,Oidx)'];
                end

                if isempty(Eidx)
                    gamma_buf_Eidx = [];
                else
                    gamma_buf_Eidx = 1 + [1:size(Eidx,1)];
                    gamma(Eidx) = gamma_buf(gamma_buf_Eidx);
                end

                if isempty(Oidx)
                    gamma_buf_Oidx = [];
                else
                    gamma_buf_Oidx = 1 + size(gamma_buf_Eidx,2) + [1:size(Oidx,1)];
                    gamma(Oidx) = gamma_buf(gamma_buf_Oidx);
                end
                gamma(cidx) = gamma_buf(1);
            end

            %%%%%%%%%%%%%%%%%%%%%% Book Keeping %%%%%%%%%%%%%%%%%%%
            %______________________________________________________
            % CASE #1
            %   c->E
            delta_alphac_alpha = C - alpha(cidx);
            if isempty(Sidx)
                delta_alphac_alpha = inf;
            end

            %______________________________________________________
            % CASE #2
            %   c->S
            delta_alphac_g = -g(cidx)/gamma(cidx);
            delta_alphac_g(find(delta_alphac_g<0)) = inf;

            %______________________________________________________
            % CASE #3
            %   S->E or S->O

            if isempty(Sidx)
                delta_alphac_S = inf;
            else
                IS_plus = Sidx(find(beta(Sidx) > eps*10));
                IS_minus = Sidx(find(beta(Sidx) < -eps*10));

                delta_alphai_max = ones(ndata,1)*inf;

                if isempty(IS_plus)
                    delta_alphai_max(IS_plus) = inf;
                else
                    delta_alphai_max(IS_plus) = (C - alpha(IS_plus))./beta(IS_plus);
                end

                if isempty(IS_minus)
                    delta_alphai_max(IS_minus) = inf;
                else
                    delta_alphai_max(IS_minus) = -alpha(IS_minus)./beta(IS_minus);
                end

                delta_alphai_max(find(delta_alphai_max<0)) = inf;

                [delta_alphac_S, IS_idx] = min(delta_alphai_max);

                if size(find(IS_plus == IS_idx),1) ~= 0
                    IS_flag = 'plus+';
                else
                    IS_flag = 'minus';
                end
            end
            %______________________________________________________
            % CASE #4
            %   E->S
            IE = Eidx(find(gamma(Eidx) > 0));
            
            delta_alphac_E = inf;

            if ~isempty(IE)
                delta_alphac_E_buf = -g(IE)./gamma(IE);
                delta_alphac_E_buf(find(delta_alphac_E_buf<=0)) = inf;
                [delta_alphac_E, IE_idx] = min(delta_alphac_E_buf);

                IE_idx = IE(find(delta_alphac_E_buf == delta_alphac_E));
            end

            %______________________________________________________
            % CASE #5
            %   O->S
            IO = Oidx(find(gamma(Oidx) < 0));

            delta_alphac_O = inf;

            if ~isempty(IO)
                delta_alphac_O_buf = -g(IO)./gamma(IO);
                delta_alphac_O_buf(find(delta_alphac_O_buf<=0)) = inf;
                [delta_alphac_O, IO_idx] = min(delta_alphac_O_buf);
                IO_idx = IO(IO_idx);
            end

            %______________________________________________________
            % Minimum delta alpha
            %
            [delta_alphac_max, CASE] = min([delta_alphac_alpha,...
                delta_alphac_g,...
                delta_alphac_S,...
                delta_alphac_E,...
                delta_alphac_O]);

            %______________________________________________________
            % UPDATE
            %
            if isempty(Sidx)
                g = g + delta_alphac_max;
                mu = mu + delta_alphac_max;
            else
                delta_mu = beta(cidx) * delta_alphac_max;
                delta_alpha_s = beta(Sidx) * delta_alphac_max;

                delta_g = gamma * delta_alphac_max;

                mu = mu + delta_mu;
                alpha(cidx) = alpha(cidx) + delta_alphac_max;
                alpha(Sidx) = alpha(Sidx) + delta_alpha_s;
                g = g + delta_g;
            end

            % filtering
            alpha(find(alpha < eps)) = 0;
            alpha(find(alpha > C-eps)) = C;

            %-------------------- move check ----------------------
            switch(CASE)
                case 1  % c -> E
                    iter = 0;

                    Eidx = [Eidx;cidx];
                    %%%%%%%%%%%%%%%%%%%%% 결과 plot %%%%%%%%%%%%%%%%%%%%%%%
                    % incSVDD_drawing(time_delay);
                    % disp('c to E');
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                case 2  % c -> S
                    iter = 0;

                    if isempty(Sidx)
                        P = [0, y(cidx)'; y(cidx), 2*Qxx(cidx,cidx)];
                        Q = inv(P);
                    else
                        Q = updateQ_add(y(cidx),Qxx(cidx,Sidx),Q);
                    end

                    Sidx = [Sidx;cidx];

                    %%%%%%%%%%%%%%%%%%%%% 결과 plot %%%%%%%%%%%%%%%%%%%%%%%
                    % incSVDD_drawing(time_delay);
                    % disp('c to S');
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                case 3
                    if IS_flag == 'minus'   % S -> O
                        Q = updateQ_removal(Q,find(Sidx == IS_idx));
                        Oidx = [Oidx;IS_idx];
                        Sidx(find(Sidx == IS_idx)) = [];

                    elseif IS_flag == 'plus+'    % S -> E
                        Q = updateQ_removal(Q,find(Sidx == IS_idx));
                        Eidx = [Eidx;IS_idx];
                        Sidx(find(Sidx == IS_idx)) = [];
                    end

                case 4  % E -> S
                    if isempty(Sidx)
                        Sidx = [Sidx;IE_idx];
                        P = [0, y(Sidx)'; y(Sidx), 2*Qxx(Sidx,Sidx)];
                        Q = inv(P);
                        Eidx(find(Eidx == IE_idx)) = [];
                    else
                        Q = updateQ_add(y(IE_idx,:),Qxx(IE_idx,Sidx),Q);
                        Sidx = [Sidx;IE_idx];
                        Eidx(find(Eidx == IE_idx)) = [];
                    end

                case 5  % O -> S
                    OSidx = IO_idx;

                    if isempty(Sidx)
                        Sidx = [Sidx;IO_idx];
                        P = [0, y(Sidx)'; y(Sidx), 2*Qxx(Sidx,Sidx)];
                        Q = inv(P);
                        Oidx(find(Oidx == IO_idx)) = [];
                    else
                        Q = updateQ_add(y(IO_idx,:),Qxx(IO_idx,Sidx),Q);
                        Sidx = [Sidx;IO_idx];
                        Oidx(find(Oidx == IO_idx)) = [];
                    end
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        SET = SETup(x,y,alpha,g,Sidx,Eidx,Oidx);
    end
end