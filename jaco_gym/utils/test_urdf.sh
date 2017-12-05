echo "Testing" $1

rosrun xacro xacro --inorder $1 > temp.urdf
echo
check_urdf temp.urdf
echo 
gz sdf -p temp.urdf | tail -n 100
rm temp.urdf
