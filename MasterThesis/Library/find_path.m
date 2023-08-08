%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FindPath

function paths=find_path(d,startPos,endPos)

load('feasiblepoints.mat')



currentPos=startPos;



paths=[startPos];
while currentPos~=endPos
    [idx,dist]=knnsearch(wristFeasible,wristFeasible(currentPos,:),'K',8000);
    
    [idxEnd,distEnd]=knnsearch(wristFeasible(idx,:),wristFeasible(endPos,:),'K',length(idx));
    
    
    distEnd=distEnd*100;
    dist=dist*100;
    
    % minimum distEnd where dist<d
    count=0;
    cont=0;
    while cont==0
        count=count+1;
        if dist(idxEnd(count))<=d+0.5
            cont=1;
            currentPos=idx(idxEnd(count));
        end
    end
    paths=[paths currentPos];
    if pdist([wristFeasible(currentPos,:);wristFeasible(paths(length(paths)-1),:)])>d+0.5
        error('err')
    end
end


