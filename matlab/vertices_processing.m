function [room_facet,room_vertices,room_norm_vector]=vertices_processing(house_facet,house_vertices,house_norm_vector)

house_facet_num=size(house_facet,1);
house_facet_new=zeros(house_facet_num,3);
house_vertices_new=zeros(3*house_facet_num,3);
house_norm_vector_new=zeros(house_facet_num,3);

for i=1:1:house_facet_num
    house_facet_new(i,:)=house_facet(i,:);
    house_vertices_new(3*i-2:3*i,1:3)=house_vertices(3*i-2:3*i,1:3);
    house_norm_vector_new(i,1:3)=house_norm_vector(i,1:3);
end

house_facet=house_facet_new;
house_vertices=zeros(3*house_facet_num,3);
house_vertices(:,1)=house_vertices_new(:,1);
house_vertices(:,2)=house_vertices_new(:,2);
house_vertices(:,3)=house_vertices_new(:,3)-min(house_vertices_new(:,3));
house_norm_vector=house_norm_vector_new;

for i=1:1:1
    for j=1:1:size(house_facet,1)
        room_facet{i}(j,1:3)=[3*j-2, 3*j-1, 3*j];
        room_vertices{i}(3*j-2:3*j,1:3)=house_vertices(3*j-2:3*j,1:3);
        room_norm_vector{i}(j,1:3)=house_norm_vector(j,1:3);
    end
end

end