function [membership, member_value]=find_pareto_frontier(input)
%function determines members of a pareto frontier
out=[];
data=unique(input,'rows');
for i = 1:size(data,1)
    
    c_data = repmat(data(i,:),size(data,1),1);
    t_data = data;
    t_data(i,:) = Inf(1,size(data,2));
    smaller_idx = c_data>=t_data;
    
    idx=sum(smaller_idx,2)==size(data,2);
    if ~nnz(idx)
        out(end+1,:)=data(i,:);
    end
end
membership = ismember(input,out,'rows');
member_value = out;
end