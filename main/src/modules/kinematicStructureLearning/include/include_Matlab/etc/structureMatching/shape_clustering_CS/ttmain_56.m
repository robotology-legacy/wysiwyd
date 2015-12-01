clear
clc
datapath = fullfile('56/');
datafilename = dir(fullfile('56/*.pgm'));
datanum = length(datafilename);
sample_num = 50;
show = 0;
alpha =300;
rank1 = [];
rank2 = [];
rank3 = [];
setCosts = [];
MatchingSet = [];
MatchSet = [];
for i = 1:datanum
    i
    JPtoEPdfi = [datapath datafilename(i).name '_JPtoEP.mat'];
    load(JPtoEPdfi);
    JPtoEPi = JPtoEP;
    JPdfi = [datapath datafilename(i).name '_JP.mat'];
    load(JPdfi);
    JPi = JP;
    PFdfi = [datapath datafilename(i).name '_PF.mat'];
    load(PFdfi);
    PFi = PF;
    MMdfi = [datapath datafilename(i).name '_MM.mat'];
    load(MMdfi);
    MMi = MM;
    EPdfi = [datapath datafilename(i).name '_EP.mat'];
    load(EPdfi);
    EPi = EP;
    PointSet{ i } = cell( 1, length( EPi ) );
    JuncSet{ i } = cell( 1, length( MMi ) );
    for nn1 = 1: length( EPi )
        PointSet{i}{nn1} = nn1;
    end
    JuncSet{i}=MMi';

    lab = floor((i-1) / 4);
    Costs = [];
    for j = 1:datanum
        if(j~=i)
            PFdfj = [datapath datafilename(j).name '_PF.mat'];
            load(PFdfj);
            PFj = PF;
            JPdfj = [datapath datafilename(j).name '_JP.mat'];
            load(JPdfj);
            JPj = JP;
            [Save_C{i,j}, Cost, Matching] = PathMatching(PFi,PFj,alpha);
            [c1,c2] = find(Matching == 1); %c1 and c2 are the corresponding end points of these two shapes
            JPtoEPdfj = [datapath datafilename(j).name '_JPtoEP.mat'];
            load(JPtoEPdfj);
            JPtoEPj = JPtoEP;
            MMdfj = [datapath datafilename(j).name '_MM.mat'];
            load(MMdfj);
            MMj = MM;
            MatchingSet = [];
            %compute the number of merge junctions,
            %num1*num2*...ClassMMi is a cell like <1*3 cell>
            ClassMMi = Choose( MMi );
            ClassMMj = Choose( MMj );
            CostSet = [];
            for inum = 1: length( ClassMMi ) %inum and jnum imply how many possibilities of junctions after merging
                for jnum = 1: length( ClassMMj )
                    JPtoEPMMi = JPtoEPi( ClassMMi{ inum } );
                    JPtoEPMMj = JPtoEPj( ClassMMj{ jnum } );
                    [ Match, Cost, Dis ] = PathMatch( JPtoEPMMi, JPtoEPMMj, alpha, c1, c2 ); %Matching is nJunctionPointsi*nJunctionPointsj
                    CostSet( inum, jnum ) = Cost;
                end
            end
            if size( CostSet, 1 ) == 1
                Row = 1;
                [ minii, Col ] = min( CostSet );
            else
                [ mini, MinNumRow ] = min( CostSet );
                [ minii, Col ] = min( mini );
                Row = MinNumRow( Col );
            end
            JPtoEPMMi = JPtoEPi( ClassMMi{ Row } );
            JPtoEPMMj = JPtoEPj( ClassMMj{ Col } );
            [ Match, Cost, Dis ] = PathMatch( JPtoEPMMi, JPtoEPMMj, alpha, c1, c2 );
            MatchDis = Match.*Dis;
            SumDis = sum( sum( MatchDis ) );
            %compare which cost is the smallest one
            [ mini, MinNum ] = min( minii );
            MatchingSet{ i, j }=Matching;
            MatchSet{ i,j }=Match;
            if(show)
                [cc1,cc2] = find(Match ==1);
                cc1 = [ ClassMMi{ Row }( cc1 ) ];
                cc2 = [ ClassMMj{ Col }( cc2 ) ];
                load([datapath datafilename(i).name '_IM.mat']);
                BWi = bwmorph(IM,'remove');
                load([datapath datafilename(j).name '_IM.mat']);
                BWj = bwmorph(IM,'remove');
                load([datapath datafilename(i).name '_SK.mat']);
                SKi = SK;
                SKi = SKMorph( SKi );
                load([datapath datafilename(j).name '_SK.mat']);
                SKj = SK;
                SKj = SKMorph( SKj );
                load([datapath datafilename(i).name '_EP.mat']);
                EPi = EP;
                load([datapath datafilename(j).name '_EP.mat']);
                EPj = EP;
                subplot( 2,1,1 );
                imshow(~(SKi+BWi));
                hold on
                for kk = 1: length( cc1 )
                    text( JPi( ( cc1( kk ) ), 2 ), JPi( ( cc1( kk ) ), 1 ), num2str( kk ),'BackgroundColor',[ 0 1 0 ] );
                end
                for kk = 1:length(c1)
                    text(EPi((c1(kk)),2), EPi((c1(kk)),1), num2str(kk),'BackgroundColor',[1 0 0]);
                end
                subplot( 2,1,2 );
                imshow( ~( SKj+BWj ) );
                hold on
                for kk = 1:length( cc2 )
                    text( JPj( ( cc2( kk ) ), 2 ), JPj( ( cc2( kk ) ), 1 ), num2str( kk ),'BackgroundColor',[ 0 1 0 ] );
                end
                for kk = 1:length(c2)
                    text(EPj((c2(kk)),2), EPj((c2(kk)),1), num2str(kk),'BackgroundColor',[1 0 0]);
                end
                close
            end
        else
            Cost = 0;
            Save_C{ i, j } = 0;
            MatchSet{i,j}=[];
            MatchingSet{ i, j }=[];
        end
        Costs = [ Costs; Cost ];
    end
    setCost = Costs';
    setCosts = [ setCosts; setCost ];
    setCosts_JP = setCosts;
end
save (['56/save56_5.mat'], 'setCosts', 'Save_C','MatchSet', 'PointSet','JuncSet');
% sum(rank1)
% sum(rank2)
% sum(rank3)