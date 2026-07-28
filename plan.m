function xp = plan(ind) 
xp=zeros(16,1);
wp=zeros(16,2);
%waypoints
%start
wp(7,1)=-3;
wp(8,1)=-1;
wp(9,1)=-2;
%car
wp(7,2)=-6;
wp(8,2)=-1;
wp(9,2)=-2;
%bicycle
wp(7,3)=-6;
wp(8,3)=-6;
wp(9,3)=-1;
%swimming pool
wp(7,4)=7.5;
wp(8,4)=-6;
wp(9,4)=-1;
%in front of the greenhouse
wp(7,5)=7.5;
wp(8,5)=6;
wp(9,5)=-1;
%in front of the tree
wp(7,6)=-5;
wp(8,6)=6;
wp(9,6)=-1;
%car again
wp(7,7)=-6;
wp(8,7)=-1;
wp(9,7)=-2;
%begginig
wp(7,8)=-3;
wp(8,8)=-1;
wp(9,8)=-2;

%switching the waypoint
xp(7)=wp(7,ind);
xp(8)=wp(8,ind);
xp(9)=wp(9,ind);

%velocity (optional)
v_x=0;
v_y=0;
v_z=0;
xp(1)=v_x;
xp(2)=v_y;
xp(3)=v_z;

end