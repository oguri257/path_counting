import sys
from argparse import ArgumentParser

############################################################
''' コマンド引数のパース '''

def get_args():
  argparser = ArgumentParser()
  argparser.add_argument('input', help='入力ファイル')
  argparser.add_argument('-dijk', type=int, default=0, help='オプション')
  argparser.add_argument('-c', type=int, default=0, help='c変数への制約')
  argparser.add_argument('-del_end', type=int, default=0, help='末端ノードの削除')
  argparser.add_argument('-st', type=int, default=0, help='始点終点の大小関係の制約')
  return argparser

##########################################################################
''' 変数の定義 '''
input_filename = sys.argv[1]
input_filename_base=(input_filename.split('.'))[0]
graph = []         #無向グラフ 隣接リスト
start_node = None  #始点（整数）：初期値None
finish_node = None #終点（整数）：初期値None
length = 0         #最大長：初期値0
n = 0              #変数の数
edges = 0          #辺の数
#dijklist=[]        #ダイクストラ法で求めた始点からの距離、インデックスがノード番号と一致していて距離が格納
oknode=[]          #(始点からの距離)-(終点からの距離)<=最大パス
overnode=[]        #(始点からの距離)-(終点からの距離)>最大パス

##########################################################################
''' ファイルの読み込み '''
def load(input_filename):
    global graph          #無向グラフ 隣接リスト
    global start_node     #始点（整数）：初期値None
    global finish_node    #終点（整数）：初期値None
    global length         #最大長：初期値0
    global n              #変数の数
    global edges
    global k

    with open(input_filename) as f:
        for line in f:
            line_block = line.split()
            if (line_block[0] == 'p'):
                n = int(line_block[2]) #変数の数
                edges = int(line_block[3]) #辺の数
                graph = [[] for _ in range(n+1)] #空の隣接リストを作成 0~nまで　インデックス0は仮のリスト

            elif (line_block[0] == 'e'):
                e_1 = int(line_block[1])
                e_2 = int(line_block[2])
                graph[e_1].append(e_2)
                graph[e_2].append(e_1)

            elif (line_block[0] == 'l'):
                length = int(line_block[1]) #length：問題の最大パス

            elif (line_block[0] == 't'):
                start_node = int(line_block[1])
                finish_node = int(line_block[2])

        k=min(n-1, length) #パス長の最大値

##########################################################################

def replace_sf(): #始点と終点の入れ替え
    global start_node
    global finish_node
    tmp_start=start_node
    start_node=finish_node
    finish_node=tmp_start
#replace()

##########################################################################
''' ダイクストラに関する関数 '''

def dijkstra(start_node,n): ##ダイクストラでソートされた順でノード番号を格納したリストを返す関数

    global graph
    #global dijklist

    node_name = [i for i in range(n+1)]     #ノードの名前を0~ノードの数で表す
    node = [float('inf')] * (n+1)
    if start_node != None:
        s = start_node
    else:
        s = 1
    node[s] = 0  #始点は距離0
    cost  = 1    #コスト（パスのコストは全て1）

    while len(node_name)>2:
        for i in node_name:
            if node[i]<node[s]:
                s=i
        min_point = node_name.pop(node_name.index(s))
        for goal in graph[min_point]:
            #更新条件
            if node[min_point] + cost < node[goal]:
                node[goal] = node[min_point] + cost     #更新
        s = node_name[1]#rは始点番号

    #dijklist=node   #dijkstraという変数にダイクストラ法で求めた距離を保持

    list=[]
    k=0

    #始点からの距離が短い順に並べてlistに格納
    while True:
        list1=[i for i, x in enumerate(node) if x == k]
        if list1 != []:
            list+=list1
            k+=1
        else:
            break
    return list

def dijkstra_simp(start_node,finish_node,n,k): ##ダイクストラで始点と終点からの最短パス長の和が最大パス長を超えていたら削除したgraphを返す

    global graph
    global oknode   #始点と終点の最短距離が最長パスを超えない集合
    global overnode #始点と終点の最短距離が最長パスを超える集合
    global node_s   #始点からの距離
    global node_t   #終点からの距離

    node_name_s = [i for i in range(n+1)]     #ノードの名前を0~ノードの数で表す:始点
    node_name_t = [i for i in range(n+1)]     #ノードの名前を0~ノードの数で表す:終点
    node_s = [float('inf')] * (n+1)           #始点からの最短パス長
    node_t = [float('inf')] * (n+1)           #終点からの最短パス長
    s = start_node    #始点
    t = finish_node   #終点
    node_s[s]=0       #始点からの最短パス集合における始点からの距離0
    node_t[t]=0       #終点からの最短パス集合における終点からの距離0
    cost  = 1         #コスト

    #始点からの各ノードへの最短パスを計算しnode_sに格納 インデックスがノード番号と一致
    while len(node_name_s)>2:
        for i in node_name_s:
            if node_s[i]<node_s[s]:
                s=i
        min_point_s = node_name_s.pop(node_name_s.index(s))
        for goal in graph[min_point_s]:
            #更新条件
            if node_s[min_point_s] + cost < node_s[goal]:
                node_s[goal] = node_s[min_point_s] + cost     #更新
        s = node_name_s[1]                                    #rは始点番号


    #終点からの各ノードへの最短パスを計算しnode_tに格納 インデックスがノード番号と一致
    while len(node_name_t)>2:
        for i in node_name_t:
            if node_t[i]<node_t[t]:
                t=i
        min_point_t = node_name_t.pop(node_name_t.index(t))
        for goal in graph[min_point_t]:
            #更新条件
            if node_t[min_point_t] + cost < node_t[goal]:
                node_t[goal] = node_t[min_point_t] + cost     #更新
        t = node_name_t[1]                                    #rは始点番号

    st_minpass=list(map(lambda x,y: x+y, node_s, node_t))     #始点と終点からの最短パスを足したもの


    for i, j in enumerate(st_minpass):
        if j>k:
            st_minpass[i]=(i,None)
            overnode.append(i)       #overnodeに到達できないノード番号を格納
        else:
            st_minpass[i]=(i,j)
            oknode.append(i)         #oknodeに到達できるノード番号を格納

    #グラフの更新
    for i, j in enumerate(graph):
        if i in overnode:
            graph[i]=None
            continue
        new_edge=[]
        for j in j:
            if j not in overnode:
                new_edge.append(j)
        graph[i]=new_edge

    return graph

def del_endnode(n,start_node,finish_node,check_list=None):
    global graph

    next_check=[]
    if check_list==None:
        for i in range(1,n+1):
            if graph[i]==None:
                continue
            elif len(graph[i])==1 and i!=finish_node and i!=start_node:#行き先が1つ　かつ　始点でも終点でもない
                tmp_edge=graph[i].pop() #唯一の行き先を取り出す　空リストになる==独立ノードにする
                graph[tmp_edge].remove(i)
                next_check.append(tmp_edge)
                graph[i]=None
    else:
        for i in check_list:
            if graph[i]==None:
                continue
            elif len(graph[i])==1 and i!=finish_node and i!=start_node:
                tmp_edge=graph[i].pop() #行き先がないので空リストになる==独立ノード
                graph[tmp_edge].remove(i)
                next_check.append(tmp_edge)
                graph[i]=None

    list(set(next_check))
    if next_check==[]:
        return graph
    else:
        del_endnode(n,start_node,finish_node,next_check)


#######################################################################################
''' 制約生成の関数 '''

def make_min():  ##始点と終点を投射変数に追加
    min="min:"
    start_end=""
    for i in range(1,n+1):
        start_end += " 1 S({}) 1 T({})".format(str(i),str(i))##始点と終点を投射変数に追加
        for j in graph[i]:
            min += " 1 R({},{})".format(str(i),str(j))##パスを投射変数に追加
    min += start_end
    min += ";"
    return min
    ##all,oneでもSTを追加

def make_min_dijk():#始点　辺　終点　ソースから順に並べるように　ダイクストラ
    global graph

    start=""  ##始点と終点を投射変数に追加
    end=""
    min_r=""
    for i in range(1,n+1):
        if i not in overnode:
            start += " 1 S({})".format(str(i))##始点と終点を投射変数に追加
            end   += " 1 T({})".format(str(i))
    for j in sortnode:
        if graph[j]!=None:
            for l in graph[j]:
                min_r += " 1 R({},{})".format(str(j),str(l))##パスを投射変数に追加
    min = "min:" + start + min_r + end + ";"
    return min
    ##all,oneでもSTを追加

def make_start():#S(i):iはノードi
    global start_node
    global n
    start_const = ""
    for i in sortnode:
        if i not in overnode:
            start_const += "1 S({}) ".format(str(i))
    start_const += "= 1;"
    return start_const

def make_finish():#T(i):iはノードi
    global finish_node
    global n
    finish_const = ""
    for i in sortnode:
        if i not in overnode:
            finish_const+="1 T({}) ".format(str(i))
    finish_const += "= 1;"
    return finish_const


I_consts=[]
O_consts=[]
def make_inedge():
    global n
    global I_consts

    I_consts = [None for _ in range(n+1)]
    for i in range(1,int(n)+1):
        Ii_const = ""
        if graph[i] != None:
            if graph[i] != []:
                for j in graph[i]:
                    Ii_const += "1 R({},{}) ".format(str(j), str(i))
                I_consts[i]=Ii_const

def make_outedge():
    global n
    global O_consts

    O_consts = [None for _ in range(n+1)]
    for i in range(1,n+1):
        Oi_const = ""
        if graph[i] != None:
            if graph[i] != []:
                for j in graph[i]:
                    Oi_const += "1 R({},{}) ".format(str(i), str(j))
                O_consts[i]=Oi_const

#I_consts,O_constsはインデックス番号+1がIiのiに対応

def make_cicj(i,j,k):#k:int型
    cicj_const=[]
    cicj_const.append("1 ~R({},{}) 1 C({},{}) >= 1;".format(str(i), str(j), str(i), str(k-1)))
    cicj_const.append("1 ~R({},{}) 1 ~C({},{}) >= 1;".format(str(i), str(j), str(j), str(0)))
    for m in range(k-1):
        cicj_const.append("1 ~R({},{}) 1 ~C({},{}) 1 C({},{}) >= 1;".format(str(i), str(j), str(j), str(m+1), str(i), str(m)))
    return  cicj_const


def make_const_st():###始点が終点よりも小さい　S(i)<T(j)
    for i in range(1,n):
        print("1 ~B({}) + 1 B({}) >= 1;".format(str(i),str(i+1)))
    for i in range(1,n+1):
        print("1 ~T({}) + 1 B({}) >= 1;".format(str(i),str(i)))
        print("1 ~B({}) + 1 ~S({}) >= 1;".format(str(i),str(i)))

#######################################################################################
''' メイン 制約の生成'''

if __name__ == '__main__':
    # 引数の解析
    args_obj = get_args()
    args = args_obj.parse_args()
    fname = args.input
    option = args.dijk
    c_const = args.c

    # ロード
    load(args.input)

    sortnode=dijkstra(start_node, n)            #sortnodeにダイクストラでの距離順にノード番号を格納

    #オプションで-dijk 0ならば簡単化しない（all_pairs用）-dijk 1　ならば簡単化(one_pair用)
    if args.dijk == 0:
        pass
    elif args.dijk == 1:
        dijkstra_simp(start_node,finish_node,n,k)  #内部表現のgraphを更新

    if args.del_end == 0:
        pass
    elif args.del_end == 1:
        del_endnode(n,start_node,finish_node)  #内部表現のgraphを末端ノードを削除して更新


    make_inedge()
    make_outedge()
    print(make_min_dijk())#ダイクストラでソースから近い順に変数を出力
    print(make_start())
    print(make_finish())

    if start_node != None:
        print("1 S({}) = 1;".format(str(start_node)))
    else:
        pass
    if finish_node != None:
        print("1 T({}) = 1;".format(str(finish_node)))
    else:
        pass


    for i in range(1,n+1):
        if graph[i] != None:
            if I_consts[i] != None:
                print(I_consts[i] + "<= 1;")
                print("d S({}) => ".format(str(i)) + I_consts[i] + "= 0;")
                print("d T({}) => ".format(str(i)) + I_consts[i] + "= 1;")
            if O_consts[i] != None:
                print(O_consts[i] + "<= 1;")
                print("d S({}) => ".format(str(i)) + O_consts[i] + "= 1;")
                print("d T({}) => ".format(str(i)) + O_consts[i] + "= 0;")
            print("d x({}) <= 1 ~S({}) 1 ~T({}) >= 2;".format(str(i), str(i), str(i)))
            print("d x({}) => ".format(str(i)) + I_consts[i] + O_consts[i].replace('1 ', '-1 ') + "= 0;")
            for m in range(k-1):
                print("1 ~C({},{}) 1 C({},{}) >= 1;".format(str(i), str(m), str(i), str(m+1)))
            for j in graph[i]:
                cicj=make_cicj(i,j,k)
                for p in cicj:
                    print(p)
            if args.c == 0:
                continue
            elif args.c == 1:
                pathlength=int(node_s[i])
                if pathlength>0:
                    print("1 C({},{}) = 0;".format(str(i), str((pathlength-1))))
                pathlength=int(node_t[i])
                if pathlength>0:
                    pathlength=k-pathlength
                    print("1 C({},{}) = 1;".format(str(i), str((pathlength))))
    if args.st == 0:
        pass
    elif args.st == 1:
        make_const_st()
