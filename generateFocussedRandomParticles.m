function [ particle_mat_ ] = generateFocussedRandomParticles( numParticles, global_map, map_resolution, free_threshold ,position)

%   this generates focussed particles



particle_mat_ = [];
while(size(particle_mat_,2)<numParticles)
    particle_mat = repmat(position,2*numParticles,1)' + [0.1*randn(2*numParticles,1) 0.1*randn(2*numParticles,1) 5*(pi/180)*randn(2*numParticles,1)]';

    index_mat = round(particle_mat/map_resolution);

    particle_mat(:,index_mat(1,:)>size(global_map,1))=[];
    particle_mat(:,index_mat(2,:)>size(global_map,1))=[];

    particle_mat(:,index_mat(1,:)<1)=[];
    particle_mat(:,index_mat(2,:)<1)=[];


    index_mat(index_mat(1,:)>size(global_map,1))=[];
    index_mat(index_mat(2,:)>size(global_map,1))=[];
    index_mat(index_mat(1,:)<1)=[];
    index_mat(index_mat(2,:)<1)=[];
    if(~isempty(index_mat))
        particle_mat(:,global_map(sub2ind(size(global_map),index_mat(1,:),index_mat(2,:)))<free_threshold)=[];
    else
        display('please pass in the right position')
        keyboard
    end
    particle_mat_ = [particle_mat_,particle_mat];
end
particle_mat_ = particle_mat_(:,1:numParticles);

end

