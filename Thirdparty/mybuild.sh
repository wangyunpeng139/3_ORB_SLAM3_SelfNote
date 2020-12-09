echo "start =====>>>g2o "
cd ./g2o/
rm -r ./build
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE ..
make -j8
cd ../
rm -r ./build
cd ..
echo "success to make the g2o"


echo "start =====>>>DBoW2 "
cd ./DBoW2/
rm -r ./build
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE ..
make -j8
cd ../
rm -r ./build
cd ..
echo "success to make the DBoW2"


echo "start =====>>>GLog "
cd ./GLog/
rm -r ./build
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE ..
make -j8
cd ../
rm -r ./build
cd ..
echo "success to make the GLog"









