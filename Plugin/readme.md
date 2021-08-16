# 如何创建Qt Plugins

Qt 包含两种类型的 API 用于扩展插件。

* 用于编写 Qt 自身扩展的高级API: 自定义数据库驱动程序、图像格式、文本编解码器、自定义样式等。
* 用于扩展 Qt 应用程序的底层API。

例如，如果您想编写一个定制的 QStyle 子类，并让 Qt 应用程序动态地加载它，您将使用高级API。

由于高级 API 构建在低级 API 之上，所以两者都存在一些共同的问题。如果您想为 Qt Designer 提供插件，请参阅 Qt Designer 模块文档。

## 高级 API

## 底层 API