#毕业设计代码

## 环境配置

### 开发环境
开发环境为Mac OS X 10.10，依赖库不能运行在Windows平台，可以尝试小幅修改之后在Ubuntu上配置。

### 依赖库

自行编译的库请先安装XCode的Command-Line-Tool的clang编译器

1. Boost
    ```brew install boost```
2. DBoW2
    参见Github：[github.com/dorian3d/DBoW2.git](https://github.com/dorian3d/DBoW2)
3. freeglut
    参见：[soureforge](http://freeglut.sourceforge.net/)自行编译
4. g2o
    参见Github：[github.com/RainerKuemmerle/g2o.git](https://github.com/RainerKuemmerle/g2o)
5. opencv
    ```brew install opencv --with-opengl --with-tbb --with-qt```
    编译版本2.4.12，需要加入带Qt核心的OpenGL渲染后端
6. openGL
    XCode的Library中自带了OpenGL.framework
7. Eigen
    ```brew install eigen```