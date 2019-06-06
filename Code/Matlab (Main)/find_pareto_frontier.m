function [membership, member_value]=find_pareto_frontier(des_pts)
%function determines members of a pareto frontier
%takes design points in as row vectors
%function returns membership, which is a vector of 1s and 0s. 1 represents
%that a point is a member of the Pareto Frontier. Member value are the
%points that is part of the Pareto Frontier
out=[];
data=unique(des_pts,'rows'); %gets rid of repeated design points
for i = 1:size(data,1)
    
    test_pt = repmat(data(i,:),size(data,1),1);
    comp_pts = data;
    comp_pts(i,:) = Inf(1,size(data,2)); %sets current point to a large value
    smaller_idx = test_pt>=comp_pts; %determines whether test point has individual 
    %component values that are greater than the whole dataset
    
    idx=sum(smaller_idx,2)==size(data,2); 
    %if any of smaller_idx is equal to dimension of data, 
    %then it won't be a member of the pareto frontier
    if ~nnz(idx)
        out(end+1,:)=data(i,:); %adds to Pareto Frontier in idx all 0
    end
end
membership = ismember(des_pts,out,'rows');
member_value = out;
end