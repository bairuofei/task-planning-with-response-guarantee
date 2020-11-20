## spot转化的buchi自动机转移条件

单个ap由“（）”包围；

与、或、非连接的命题，也由“（）”包围。

## networkx模块
list(graph.nodes.data(attribute_name))  返回一个tuple list,第一个元素为node name, 第二个元素为node attribute
edge与此同理

## graphviz语法

graphviz提供两个类：Graph and Digraph

dot.node(str1, str2)
str1: name identifier
str2: label

dot.edge

dot.render(file_pos, view = True)
view = True将自动打开保存的文件


## re模块
re.match只匹配字符串的开始，如果字符串开始不符合正则表达式，则匹配失败；
re.search匹配整个字符串，直到找到一个匹配

re.compile函数用于编译正则表达式，生成一个正则表达式(Pattern)对象，供match()和search()两个函数使用。

## python
list[::-1]: 逆序排列的原列表