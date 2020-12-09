###
 # @Description: 
 # @Version: 1.0
 # @Autor: skylor
 # @Date: 2020-08-08 21:58:01
 # @LastEditors: skylor
 # @LastEditTime: 2020-08-08 22:28:27
### 
# Euroc
pathDatasetEuroc='/home/wyp/myextern/dataset/EuRoC' #Example, it is necesary to change it by the dataset path

#MH03完整测试下来没问题,最后一个参数是生成txt记录文件的文件名
echo "Launching MH03 with Monocular-Inertial sensor"
./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH03 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH03.txt dataset-MH03_monoi

