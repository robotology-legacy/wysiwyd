function SETupdate(update_CASE, x, y, alpha, g, idx, ref_idx)

global SET

switch update_CASE
    case 'c->S'
        SET.S.x = [SET.S.x;x(idx,:)];
        SET.S.y = [SET.S.y;y(idx)];
        SET.S.alpha = [SET.S.alpha;alpha(idx)];
        SET.S.g = [SET.S.g;g(idx)];
        SET.S.ndata = SET.S.ndata + 1;        

    case 'c->E'
        SET.E.x = [SET.E.x;x(idx,:)];
        SET.E.y = [SET.E.y;y(idx)];
        SET.E.alpha = [SET.E.alpha;alpha(idx)];
        SET.E.g = [SET.E.g;g(idx)];
        SET.E.ndata = SET.E.ndata + 1;

    case 'c->O'
        SET.O.x = [SET.O.x;x(idx,:)];
        SET.O.y = [SET.O.y;y(idx)];
        SET.O.alpha = [SET.O.alpha;alpha(idx)];
        SET.O.g = [SET.O.g;g(idx)];
        SET.O.ndata = SET.O.ndata + 1;

    case 'S->E'
        sidx = find(ref_idx == idx);
        SET.E.x = [SET.E.x;x(idx,:)];
        SET.E.y = [SET.E.y;y(idx,:)];
        SET.E.alpha = [SET.E.alpha;alpha(idx)];
        SET.E.g = [SET.E.g;g(idx)];
        SET.E.ndata = SET.E.ndata + 1;
        
        SET.S.x = SET.S.x([1:sidx-1,sidx+1:end],:);
        SET.S.y = SET.S.y([1:sidx-1,sidx+1:end],:);
        SET.S.alpha = SET.S.alpha([1:sidx-1,sidx+1:end]);
        SET.S.g = SET.S.g([1:sidx-1,sidx+1:end]);
        SET.S.ndata = SET.S.ndata - 1;

    case 'S->O'
        sidx = find(ref_idx == idx);
        SET.O.x = [SET.O.x;x(idx,:)];
        SET.O.y = [SET.O.y;y(idx,:)];
        SET.O.alpha = [SET.O.alpha ; alpha(idx)];
        SET.O.g = [SET.O.g;g(idx)];
        SET.O.ndata = SET.O.ndata + 1;
        
        SET.S.x = SET.S.x([1:sidx-1,sidx+1:end],:);
        SET.S.y = SET.S.y([1:sidx-1,sidx+1:end],:);
        SET.S.alpha = SET.S.alpha([1:sidx-1,sidx+1:end]);
        SET.S.g = SET.S.g([1:sidx-1,sidx+1:end]);
        SET.S.ndata = SET.S.ndata - 1;

    case 'E->S'
        eidx = find(ref_idx == idx);
        SET.S.x = [SET.S.x;x(idx,:)];
        SET.S.y = [SET.S.y;y(idx,:)];
        SET.S.alpha = [SET.S.alpha ; alpha(idx)];
        SET.S.g = [SET.S.g;g(idx)];
        SET.S.ndata = SET.S.ndata + 1;
        
        SET.E.x = SET.E.x([1:eidx-1,eidx+1:end],:);
        SET.E.y = SET.E.y([1:eidx-1,eidx+1:end],:);
        SET.E.alpha = SET.E.alpha([1:eidx-1,eidx+1:end]);
        SET.E.g = SET.E.g([1:eidx-1,eidx+1:end]);
        SET.E.ndata = SET.E.ndata - 1;
        
    case 'O->S'
        oidx = find(ref_idx == idx);
        SET.S.x = [SET.S.x;x(idx,:)];
        SET.S.y = [SET.S.y;y(idx,:)];
        SET.S.alpha = [SET.S.alpha ; alpha(idx)];
        SET.S.g = [SET.S.g;g(idx)];
        SET.S.ndata = SET.S.ndata + 1;
        
        SET.O.x = SET.O.x([1:oidx-1,oidx+1:end],:);
        SET.O.y = SET.O.y([1:oidx-1,oidx+1:end],:);
        SET.O.alpha = SET.O.alpha([1:oidx-1,oidx+1:end]);
        SET.O.g = SET.O.g([1:oidx-1,oidx+1:end]);
        SET.O.ndata = SET.O.ndata - 1;
end

