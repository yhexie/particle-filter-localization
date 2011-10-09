function [map_xy,map_size,auto_shift,map_dim,resolution]  = readmap(map_str)
fid = fopen(map_str);
map_xy = [];
count = 0;
tline = fgetl(fid);
count = count+1;
while(ischar(tline))
   if(count ==1) 
       global_mapsize_x = str2num(tline(50:end))
   end
   
   if(count ==2) 
       global_mapsize_y = str2num(tline(50:end))
   end
   
   if(count ==3) 
       resolution = str2num(tline(50:end))
   end
   
   if(count ==4) 
       autoshifted_x = str2num(tline(50:end))
   end
   
   if(count ==5) 
       autoshifted_y = str2num(tline(50:end))
   end
      
   if(count ==7) 
       map_dim = str2num(tline(15:end))
   end
   
   if(count>7) 
       map_xy = [map_xy;str2num(tline)];
   end
      
   tline = fgetl(fid);
   count = count+1;
      
end

map_size = [global_mapsize_x,global_mapsize_y];
auto_shift = [autoshifted_x,autoshifted_y];

fclose(fid);