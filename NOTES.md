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





{'s1': [0, 1, 2, 3], 's2': [0, 1, 2], 's4': [0, 3], 's3': [0, 1, 2, 3]})
[{'s1': 25, 's2': 32, 's4': 41, 's3': 45}, 
{'s1': 59, 's2': 66, 's3': 77}, 
{'s1': 25, 's2': 32, 's3': 43}, 
{'s1': 25, 's4': 35, 's3': 39}]


['s1', 's2', 's4', 's3']

75 77 79 71

[109, 79, 115, 111]

s1:59

addtion
1 24  34  (75) 34  (79)  34
2 0   0            (77)  2
3 24  34           (77)  36
4 24      (59) 40   (79) 40
































