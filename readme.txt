这个是我把自己以前写的slam的运行环境改了的。其中原来有调用 g20， eigen3 ，PCL 等一些库的链接。但是文件已经被我删了，并没有不影响我们的实验。

主要代码在src 中 

打开opticalFLow.cpp 就可以编辑 
然后保存， 执行一下命令
1.  cd ~/optical_flow/build 
2.  cmake ..
3.  make
4.  cd ~/optical_flow/bin
5.  ./opticalFlow
然后就可以执行了

添加和创建新的可执行文件
详细可以打开 src/CMakeList.txt

ADD_EXECUTABLE(opticalFlow  opticalFlow.cpp)
TARGET_LINK_LIBRARIES( opticalFlow
	${OpenCV_LIBS}  
	${PCL_LIBRARIES})

只要将 以上代码中 opticalFLow  换成新的cpp 的文件就行了。。


 
