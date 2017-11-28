echo "Testing" $1

rosrun xacro xacro --inorder $1 > temp.urdf
echo
check_urdf temp.urdf
echo 
gz sdf -p temp.urdf | head -n 30
rm temp.urdf