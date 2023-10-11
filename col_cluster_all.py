#! /usr/bin/env python3

import subprocess
import time
import math
import sys
import copy
from argparse import ArgumentParser
from networkx.algorithms import community
import networkx as nx
import itertools
import tempfile
import multiprocessing

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
    dijk_min = "min:" + start + min_r + end + ";"
    return dijk_min
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

def make_one_const():
    global n
    global edges
    global start_node
    global finish_node
    global k
    global dijk_min

    consts=[]

    I_consts = [None for _ in range(n+1)]
    for i in range(1,int(n)+1):
        Ii_const = ""
        if graph[i] != None:
            if graph[i] != []:
                for j in graph[i]:
                    Ii_const += "1 R({},{}) ".format(str(j), str(i))
                I_consts[i]=Ii_const
    O_consts = [None for _ in range(n+1)]
    for i in range(1,n+1):
        Oi_const = ""
        if graph[i] != None:
            if graph[i] != []:
                for j in graph[i]:
                    Oi_const += "1 R({},{}) ".format(str(i), str(j))
                O_consts[i]=Oi_const
    consts.append(make_min_dijk())
    consts.append(make_start())
    consts.append(make_finish())

    if start_node != None:
        consts.append("1 S({}) = 1;".format(str(start_node)))
    else:
        pass
    if finish_node != None:
        consts.append("1 T({}) = 1;".format(str(finish_node)))
    else:
        pass


    for i in range(1,n+1):
        if graph[i] != None:
            if I_consts[i] != None:
                consts.append(I_consts[i] + "<= 1;")
                consts.append("d S({}) => ".format(str(i)) + I_consts[i] + "= 0;")
                consts.append("d T({}) => ".format(str(i)) + I_consts[i] + "= 1;")
            if O_consts[i] != None:
                consts.append(O_consts[i] + "<= 1;")
                consts.append("d S({}) => ".format(str(i)) + O_consts[i] + "= 1;")
                consts.append("d T({}) => ".format(str(i)) + O_consts[i] + "= 0;")
            consts.append("d x({}) <= 1 ~S({}) 1 ~T({}) >= 2;".format(str(i), str(i), str(i)))
            consts.append("d x({}) => ".format(str(i)) + I_consts[i] + O_consts[i].replace('1 ', '-1 ') + "= 0;")
            for m in range(k-1):
                consts.append("1 ~C({},{}) 1 C({},{}) >= 1;".format(str(i), str(m), str(i), str(m+1)))
            for j in graph[i]:
                cicj=make_cicj(i,j,k)
                for p in cicj:
                    consts.append(p)

            pathlength=int(node_s[i])
            if pathlength>0:
                consts.append("1 C({},{}) = 0;".format(str(i), str((pathlength-1))))
            pathlength=int(node_t[i])
            if pathlength>0:
                pathlength=k-pathlength
                consts.append("1 C({},{}) = 1;".format(str(i), str((pathlength))))


    NODES=str(n)             #変数の数
    EDGES=str(edges*2)
    l="l {}".format(k)
    t="t {} {}".format(str(start_node),str(finish_node))
    return consts, NODES, EDGES, t, l

############################################################
''' グラフ読み込み '''
def load_graph(graph,n,input_filename_base=''):
  global input_filename
  G = nx.Graph()
  for i in range(1,n+1):
      if graph[i]==None or graph[i]==[]:
          continue
      else:
          G.add_node(int(i))
          for j in graph[i]:
              G.add_edge(int(i),int(j))

  pos = nx.spring_layout(G, seed=0) #ノードの配置を指定
  pos_xy = list(pos.values())

  # クラスタリング
  communities = community.greedy_modularity_communities(G)

  global cluster_list
  global node_cluster
  global cluster_endnode
  global cluster_edge
  global cluster_edge_2
  global cluster_edge_3
  global cluster_num

  cluster_num=len(communities)
  if cluster_num == 1:
    return False, graph
  # クラスタリング結果の表示と分類
  cluster_list = [[] for _ in range(cluster_num+1)] #クラスタ集合:インデックスがクラスタ番号、中身がそのノード
  node_cluster=[None]*(n+1) #インデックスがノード番号、値がクラスタ番号
  cluster_endnode=[[] for _ in range(cluster_num+1)] #クラスタ内の端ノード
  cluster_edge = [[] for _ in range(cluster_num+1)]
  cluster_edge_2 = [[] for _ in range(n+1)]
  cluster_edge_3 = []

  #graph_clusterにクラスタ番号を格納
  for i, c in enumerate(communities):
      cluster_list[i+1] = list(c)
      for k in c:
        if graph[k]==None:
          pass
        else:
          node_cluster[k]=i+1

  #クラスタを繋ぐエッジ集合とクラスタ内の端ノード集合の作成
  for i in range(1,n+1):
    if graph[i]==None:
      continue
    else:
      for j in graph[i]:#jはiの行き先
        if node_cluster[i]!=node_cluster[j]:
          cluster_endnode[node_cluster[i]].append(i)
          cluster_endnode[node_cluster[j]].append(j)
          cluster_edge[node_cluster[i]].append((i,j))
          cluster_edge[node_cluster[j]].append((j,i))
          if i <= j:
              if (i,j) not in cluster_edge_3:
                  cluster_edge_3.append((i,j))
          if j not in cluster_edge_2[i]:
              cluster_edge_2[i].append(j)
          if i not in cluster_edge_2[j]:
              cluster_edge_2[j].append(i)

  #重複を取り除く
  for i,c in enumerate(cluster_endnode):
    cluster_endnode[i]=list(set(c))
  for i,c in enumerate(cluster_edge):
    cluster_edge[i]=list(set(c))

  #クラスタ集合ごとの特徴を調査
  memo=[]
  ave_dens=0
  for v, cluster in enumerate(communities):
    #################################
    #print('クラスタ集合')
    #print(cluster)
    G_cluster = nx.Graph()
    for i in cluster:
      G_cluster.add_node(int(i))
      for j in graph[i]:
        #if (int(i),int(j)) not in list(cluster_edge):
        if (int(i),int(j)) not in cluster_edge[node_cluster[i]]:
          G_cluster.add_edge(int(i),int(j))
    #################################
    #print('クラスタ集合の特徴')
    #print('変数の数,辺の数,パスの長さ,連結成分数,密度平均,クラスタ係数,推移性,媒介中心性,固有ベクトル中心性')
    dens=printFeatures(G_cluster)
    ave_dens+=dens
    memo.append('cluster{} : {}'.format(str(v+1),str(dens)))
    #print('クラスタ集合内のエッジ')
    #print(G_cluster.edges())
  ave_dens=ave_dens/len(communities)


  if ave_dens < 0.7:
    return False, graph
  else:
    return True, graph, cluster_list, node_cluster, cluster_endnode, cluster_edge, cluster_edge_2

############################################################
"クラスタ集合の処理"
def proc_cluster(start_node,finish_node):
    global graph
    global cluster_list
    global node_cluster
    global cluster_end
    global cluster_edge
    global positive_path
    global start_cluster
    global finish_cluster

    for i, cluster in enumerate(cluster_list):
        if start_node in cluster:
            start_cluster = i
        else:
            pass
        if finish_node in cluster:
            finish_cluster = i
        else:
            pass


    tmp_cluster_edge=copy.deepcopy(cluster_edge)
    for i in range(1,cluster_num+1):
        find_path(i,tmp_cluster_edge)


positive_path=[]

###深さ優先でクラスタ同士を繋ぐことができるパスを順に見つける関数
def find_path(start_cluster,edge_list, ans_list=[], node_list=[], rev_edge=[],p=0):
    global positive_path
    global cluster_edge
    global start_node
    global finish_node
    global k
    #node_list　選んだノード
    #ans_list 今までに通ったノード

    for i, s in enumerate(edge_list[start_cluster]):
        i_node=s[0]
        j_node=s[1]
        if i==len(edge_list[start_cluster])-1:#最後のチェック
            #反対向きのパスを選んでいるかどうか
            if s in rev_edge:
                node_list.pop()
                node_list.pop()
                rev_edge.pop()

                continue

            #ノードを選んだことがあるとき（1回目でないとき）
            if node_list!=[]:
                #クラスタ内のノードを経由しない場合
                if i_node == node_list[-1]:
                    pass
                #すでにノードを選んでいる場合
                elif i_node in node_list:
                    node_list.pop()
                    node_list.pop()
                    rev_edge.pop()

                    continue

            #現在選んだパスの長さがパス長上限を超えるか
            l=p
            if ans_list==[]:#初めてのパス
                l+=1
            else:#2回目以降
                if ans_list[-1][1]==i_node:
                    l+=1
                else:
                    l+=2
            if l>k:
                node_list.pop()
                node_list.pop()
                rev_edge.pop()
                continue

            if j_node in node_list:
                node_list.pop()
                node_list.pop()
                rev_edge.pop()
                continue
            #クラスタ内の最後のパス選択

            node_list.append(i_node)
            node_list.append(j_node)
            rev_edge.append(s[::-1])
            tmp_ans_list=copy.deepcopy(ans_list)
            tmp_ans_list.append(s)
            path_end=node_cluster[j_node]#選んだパスの行き先のクラスタ番号

            if tmp_ans_list[0][0] <= j_node:
                #print('同値')
                #print(tmp_ans_list,tmp_ans_list[0][0],j_node)
                positive_path.append(tmp_ans_list)
                #print(positive_path)
                #print(tmp_ans_list,l)
            #次のクラスタでの探索開始
            find_path(path_end,edge_list,tmp_ans_list,node_list,rev_edge,l)
            #現在のクラスタは探索終了なので探索済みから削除する
            if node_list != []:
                node_list.pop()
                node_list.pop()
                rev_edge.pop()


        else:#最後以外のチェック
            #ノードを選んだことがあるとき（1回目でないとき）
            if node_list!=[]:
                if i_node == node_list[-1]:
                    pass
                #すでにノードを選んでいる場合
                elif i_node in node_list:
                    continue

            #現在選んだパスの長さがパス長上限を超えるか
            l=p
            if ans_list==[]:
                l+=1
            else:
                if ans_list[-1][1]==i_node:
                    l+=1
                else:
                    l+=2
            if l>k:
                continue

            if j_node in node_list:
                continue
            #選んだパスの反対向きのパスを選んでいたとき
            if s in rev_edge:
                continue

            #パスを選べるとき
            node_list.append(i_node)
            node_list.append(j_node)
            rev_edge.append(s[::-1])
            tmp_ans_list=copy.deepcopy(ans_list)
            tmp_ans_list.append(s)
            path_end=node_cluster[j_node]#選んだパスの行き先のクラスタ番号
            #終点を含むクラスタに到達していたらゴールパスに追加
            #if path_end == finish_cluster:
                #print('同値')
            if tmp_ans_list[0][0] <= j_node:
                positive_path.append(tmp_ans_list)
                #print(positive_path)
                #print(tmp_ans_list,l)
            #次のクラスタでの探索開始
            find_path(path_end,edge_list,tmp_ans_list,node_list,rev_edge,l)


def find_path2():###pb制約生成関数(クラスタ同士を繋ぐエッジの選び方を求めるpb制約)
    global graph
    global cluster_edge
    global cluster_edge_2
    global cluster_endnode
    global cluster_list
    global n
    global k
    global start_node
    global finish_node
    global start_cluster
    global finish_cluster

    print("cluster_edge", cluster_edge)
    print("cluster_edge_2",cluster_edge_2)
    print("cluster_endnode",cluster_endnode)
    print("cluster_list",cluster_list)
    print('start',start_node)

    I_cluster_consts = ["" for _ in range(n+1)]
    O_cluster_consts = ["" for _ in range(n+1)]
    step_cluster_consts = ["" for _ in range(n+1)]
    min_r=""
    cicj_cluster_const=[]
    start_const=""
    finish_const=""
    #始点と終点をクラスタ集合に含める
    cluster_endnode_st=copy.deepcopy(cluster_endnode)
    in_s=False
    in_t=False
    for i in cluster_endnode_st:
        if start_node in i:
            in_s=True
        if finish_node in i:
            in_t=True
    if in_s==False:
        cluster_endnode_st[start_cluster].append(start_node)
    if in_t==False:
        cluster_endnode_st[finish_cluster].append(finish_node)
    ##########################
    #Ii Oi 投射変数 ci<cj
    #クラスタの完全グラフ部分
    for cluster in cluster_endnode_st:
        for pair in itertools.permutations(cluster, 2):
            print(pair)
            i=pair[0]
            j=pair[1]
            O_cluster_consts[j]+="1 R({},{}) ".format(str(j), str(i))
            I_cluster_consts[j]+="1 R({},{}) ".format(str(i), str(j))
            step_cluster_consts[i]+="1 R({},{}) 1 R({},{}) ".format(str(i), str(j), str(j), str(i))
            min_r += " 1 R({},{})".format(str(i),str(j))##パスを投射変数に追加
            cicj_cluster_const.append("1 ~R({},{}) 1 C({},{}) >= 1;".format(str(i), str(j), str(i), str(k-1)))
            cicj_cluster_const.append("1 ~R({},{}) 1 ~C({},{}) >= 1;".format(str(i), str(j), str(j), str(0)))
            for m in range(k-1):
                cicj_cluster_const.append("1 ~R({},{}) 1 ~C({},{}) 1 C({},{}) >= 1;".format(str(i), str(j), str(j), str(m+1), str(i), str(m)))

    #クラスタ連結部分
    for i,c in enumerate(cluster_edge_2):
        if c!=[]:
            for j in c:
                O_cluster_consts[i]+="1 R({},{}) ".format(str(i), str(j))
                I_cluster_consts[i]+="1 R({},{}) ".format(str(j), str(i))
                min_r += " 1 R({},{})".format(str(i),str(j))##パスを投射変数に追加
                cicj_cluster_const.append("1 ~R({},{}) 1 C({},{}) >= 1;".format(str(i), str(j), str(i), str(k-1)))
                cicj_cluster_const.append("1 ~R({},{}) 1 ~C({},{}) >= 1;".format(str(i), str(j), str(j), str(0)))
                for m in range(k-1):
                    cicj_cluster_const.append("1 ~R({},{}) 1 ~C({},{}) 1 C({},{}) >= 1;".format(str(i), str(j), str(j), str(m+1), str(i), str(m)))

    print('I_cluster_consts',I_cluster_consts)
    print('O_cluster_consts',O_cluster_consts)
    ##########################

    #制約生成
    #投射変数
    print('min:'+min_r+';')
    #各ノードiについて
    for cluster in cluster_endnode_st:
        for i in cluster:
            print(I_cluster_consts[i] + "<= 1;")
            print("d S({}) => ".format(str(i)) + I_cluster_consts[i] + "= 0;")
            print("d T({}) => ".format(str(i)) + I_cluster_consts[i] + "= 1;")
            print(O_cluster_consts[i] + "<= 1;")
            print("d S({}) => ".format(str(i)) + O_cluster_consts[i] + "= 1;")
            print("d T({}) => ".format(str(i)) + O_cluster_consts[i] + "= 0;")
            print("d x({}) <= 1 ~S({}) 1 ~T({}) >= 2;".format(str(i), str(i), str(i)))
            print("d x({}) => ".format(str(i)) + I_cluster_consts[i] + O_cluster_consts[i].replace('1 ', '-1 ') + "= 0;")
            for m in range(k-1):
                print("1 ~C({},{}) 1 C({},{}) >= 1;".format(str(i), str(m), str(i), str(m)))
            start_const  += "1 S({}) ".format(str(i))
            finish_const += "1 T({}) ".format(str(i))
    start_const  += "= 1;"
    finish_const += "= 1;"
    #各エッジについて
    for i in cicj_cluster_const:
        print(i)
    #始点と終点について
    print(start_const)
    print(finish_const)
    print("1 S({}) = 1;".format(str(start_node)))
    print("1 T({}) = 1;".format(str(finish_node)))
    #各クラスタ内で1ステップで移動する制約
    for i in step_cluster_consts:
        if i!="":
            print(i+"<= 1;")

check_list={}
answer_list=[]
#クラスタ間のエッジ順に対して各クラスタ内でのノードを辿る順番を返す関数
def find_path3(positive_path):
    global check_list
    global cluster_num
    global k
    global answer_list

    start_cluster=node_cluster[positive_path[0][0]]
    finish_cluster=node_cluster[positive_path[-1][1]]
    st_cluster=[[] for _ in range(cluster_num+1)]
    st_cluster[start_cluster].append((-1,))
    l=k-len(positive_path)
    #print(l)
    counter=[l]*(cluster_num+1)#各チェックリストでの最大パス保存
    for path in positive_path:
        i=path[0]#連結パスの始点
        j=path[1]#連結パスの終点
        #クラスタ内での使用する始点終点の追加
        #最初
        if st_cluster[node_cluster[i]]==[]:
            st_cluster[node_cluster[i]].append((i,))
        elif len(st_cluster[node_cluster[i]][-1])==1:
            if st_cluster[node_cluster[i]][-1][0]<i:
                st_cluster[node_cluster[i]][-1]+=(i,)
            else:
                st_cluster[node_cluster[i]][-1]=(i,)+st_cluster[node_cluster[i]][-1]
            #仮始点と仮終点が同じでない場合
            if st_cluster[node_cluster[i]][-1][0]!=st_cluster[node_cluster[i]][-1][1]:
                counter = [counter[x]-1 if x!=node_cluster[i] else counter[x] for x in range(cluster_num+1)]

        elif len(st_cluster[node_cluster[i]][-1])==2:
            st_cluster[node_cluster[i]].append((i,))

        if st_cluster[node_cluster[j]]==[]:
            st_cluster[node_cluster[j]].append((j,))
        elif len(st_cluster[node_cluster[j]][-1])==1:
            if st_cluster[node_cluster[j]][-1][0]<j:
                st_cluster[node_cluster[j]][-1]+=(j,)
            else:
                st_cluster[node_cluster[j]][-1]=(j,)+st_cluster[node_cluster[j]][-1]
            #仮始点と仮終点が同じでない場合
            if st_cluster[node_cluster[j]][-1][0]!=st_cluster[node_cluster[j]][-1][1]:
                counter = [counter[x]-1 if x!=node_cluster[j] else counter[x] for x in range(cluster_num+1)]

        elif len(st_cluster[node_cluster[i]][-1])==2:
            st_cluster[node_cluster[j]].append((j,))
    st_cluster[finish_cluster][-1]+=(-1,)
    if st_cluster[finish_cluster][-1][0]!=st_cluster[finish_cluster][-1][1]:
        counter = [counter[x]-1 if x!=finish_cluster else counter[x] for x in range(cluster_num+1)]


    st_cluster[0].append(k-len(positive_path))

    no_node=[]
    start_j=st_cluster[start_cluster][0][1]
    finish_i=st_cluster[finish_cluster][-1][0]
    if len(st_cluster[start_cluster])!=1:
        for i in range(1,len(st_cluster[start_cluster])):
            no_node.append(st_cluster[start_cluster][i][0])
            no_node.append(st_cluster[start_cluster][i][1])
    if len(st_cluster[finish_cluster])!=1:
        for i in range(0,len(st_cluster[finish_cluster])-1):
            no_node.append(st_cluster[finish_cluster][i][0])
            no_node.append(st_cluster[finish_cluster][i][1])

    for i in cluster_list[start_cluster]:
        tmp_counter=copy.deepcopy(counter)
        tmp_st_cluster=copy.deepcopy(st_cluster)
        #print(i,no_node)
        if i == start_j:
            tmp_counter = [tmp_counter[x]+1 if x!=node_cluster[i] else tmp_counter[x] for x in range(cluster_num+1)]
        else:
            if i in no_node:
                continue
        if i<=start_j:
            tmp_st_cluster[start_cluster][0]=(i,start_j)
        else:
            tmp_st_cluster[start_cluster][0]=(start_j,i)
        for j in cluster_list[finish_cluster]:
            tmp_tmp_counter=copy.deepcopy(tmp_counter)
            tmp_tmp_st_cluster=copy.deepcopy(tmp_st_cluster)
            #if start_cluster==finish_cluster:
            if j == finish_i:
                tmp_tmp_counter = [tmp_tmp_counter[x]+1 if x!=node_cluster[j] else tmp_tmp_counter[x] for x in range(cluster_num+1)]
            else:
                if (j in no_node) or (i==j):
                    continue
            if j<=finish_i:
                tmp_tmp_st_cluster[finish_cluster][-1]=(j,finish_i)
            else:
                tmp_tmp_st_cluster[finish_cluster][-1]=(finish_i,j)
            answer_list.append(tmp_tmp_st_cluster)
            #print('c',tmp_tmp_st_cluster,i,j)

    #クラスタのパスごとに使用できる最大パス長上限を求める
    #check_list:辞書型　{(使用するパスのタプルの集合（タプル）) : [ クラスタ内の最大パス長上限, クラスタ番号 ]}
            for i_i in range(1,cluster_num+1):
                tmp_sort=tmp_tmp_st_cluster[i_i]
                #print(tmp_sort)
                tmp_sort.sort()
                check = tuple(tmp_sort)
                if check not in check_list:
                    check_list[check]=[tmp_tmp_counter[i_i],i_i]
                else:
                    if tmp_tmp_counter[i_i] > check_list[check][0]:
                        check_list[check]=[tmp_tmp_counter[i_i],i_i]
    #return answer_list

#全体グラフからクラスタ同士を繋ぐエッジを消したグラフを生成する関数
def make_cluster_graph():
    global graph
    global cluster_edge
    global cluster_graph

    cluster_graph=copy.deepcopy(graph)
    for edge_list in cluster_edge:
        for path in edge_list:
            cluster_graph[path[0]].remove(path[1])

    return cluster_graph

#クラスタの制約生成の際に共通部分の制約の生成
def make_common_const():
    global cluster_graph#全体グラフからクラスタ連結部分を取り除いたグラフ
    global cluster_list
    global n
    global k

    global I_cluster_consts
    global O_cluster_consts
    global min_cluster
    global cm_const
    global start_const
    global finish_const

    cluster_n_1=len(cluster_list)
    cluster_const = [[] for _ in range(cluster_n_1)]
    I_cluster_consts = [[] for _ in range(n+1)]
    O_cluster_consts = [[] for _ in range(n+1)]
    min_cluster=[[] for _ in range(cluster_n_1)]
    cicj_cluster_const=[]#!!!!!#
    cm_const=[[[]for _ in range(k-1)] for _ in range(n+1)]###[ [ ],[1:クラスタ[m=0],[m=1],...,[m=k-2] ],[2:クラスタ[m=0],[m=1],...,[m=k-2] ],...,[n:クラスタ[m=0],[m=1],...,[m=k-2]] ]

    ##########################
    #Ii Oi 投射変数 ci<cj
    #クラスタの完全グラフ部分
    for c_i,cluster in enumerate(cluster_list):#c_i:クラスタ番号　
        for i in cluster:
            for j in cluster_graph[i]:
                O_cluster_consts[i].append("1 R({},{}) ".format(str(i), str(j)))
                I_cluster_consts[i].append("1 R({},{}) ".format(str(j), str(i)))
                min_cluster[c_i].append(" 1 R({},{})".format(str(i),str(j)))##パスを投射変数に追加
            for m in range(k-1):
                cm_const[i][m]=("1 ~C({},{}) 1 C({},{}) >= 1;".format(str(i), str(m), str(i), str(m+1)))


#((17, 20), (19, 18))みたいな使うパスのタプル、クラスタ内の最長パス、クラスタ番号を引数にもらって制約を出力する
#check_list {使うパスのタプル : [クラスタ内の最長パス,クラスタ番号]}　の辞書型なので　check_list　の中身を渡せば良い
def cluster_const(path_list,k,cluster_num):
    global cluster_list     #クラスタごとの集合　index:クラスタ番号 中身:クラスタに含まれるノード番号
    global cluster_graph    #全体のグラフからクラスタ連結パスを除去したグラフ
    global cluster_edge_2   #クラスタの連結パスの隣接リスト
    global n                #変数の数

    global start_node
    global finish_node
    global start_cluster
    global finish_cluster

    global I_cluster_consts
    global O_cluster_consts
    global min_cluster
    global cm_const

    c_k=k
    tmp_path_list=list(path_list)
    not_use_node=[]
    use_path=[]
    step_list=[]
    I_list=copy.deepcopy(I_cluster_consts)
    O_list=copy.deepcopy(O_cluster_consts)
    min=copy.deepcopy(min_cluster[cluster_num])
    start_const=""
    finish_const=""
    constrain=[]

    NODE=n
    EDGE=0
    for i in cluster_list[cluster_num]:
        EDGE+=len(cluster_graph[i])
    l="l {}".format(k)

    for no_step in tmp_path_list:
        #入ってすぐ出るノードと端の始点終点を取り除く
        if no_step[0]==no_step[1]:
            i=no_step[0]

            not_use_node.append(i)
            #繋がりのあるノードからも消す
            for j in cluster_graph[i]:#i,j : iは消えるノード
                if "1 R({},{}) ".format(str(i), str(j)) in I_list[j]:
                    I_list[j].remove("1 R({},{}) ".format(str(i), str(j)))
                if "1 R({},{}) ".format(str(j), str(i)) in O_list[j]:
                    O_list[j].remove("1 R({},{}) ".format(str(j), str(i)))
                if " 1 R({},{})".format(str(i), str(j)) in min:
                    min.remove(" 1 R({},{})".format(str(i), str(j)))
                if " 1 R({},{})".format(str(j), str(i)) in min:
                    min.remove(" 1 R({},{})".format(str(j), str(i)))
                continue
    for i in not_use_node:
        tmp_path_list.remove((i,i))

    if len(tmp_path_list)==0:
        return ["no_constrain"]###0ステップ

    elif k==0:
        return ["no_answer"], False###パスはない

    elif len(tmp_path_list)==1:
        clus_start_node =tmp_path_list[0][0]
        clus_finish_node=tmp_path_list[0][1]

        ###制約生成#######################################
        #投射変数
        min="".join(min)
        constrain.append(('min:'+min+';'))
        #各ノードに対する制約
        for i in cluster_list[cluster_num]:
            #使わないノードの場合は制約を生成しない
            if i not in not_use_node:
                Ii="".join(I_list[i])
                Oi="".join(O_list[i])
                constrain.append((Ii+"<= 1;"))
                constrain.append(("d S({}) => ".format(str(i)) + Ii + "= 0;"))
                constrain.append(("d T({}) => ".format(str(i)) + Ii + "= 1;"))
                constrain.append((Oi+"<= 1;"))
                constrain.append(("d S({}) => ".format(str(i)) + Oi + "= 1;"))
                constrain.append(("d T({}) => ".format(str(i)) + Oi + "= 0;"))
                constrain.append(("d x({}) <= 1 ~S({}) 1 ~T({}) >= 2;".format(str(i), str(i), str(i))))
                constrain.append(("d x({}) => ".format(str(i)) + Ii + Oi.replace('1 ', '-1 ') + "= 0;"))
                for m in range(k-1):
                    constrain.append((cm_const[i][m]))

                #各辺に対する制約
                for j in cluster_graph[i]:
                    if j not in not_use_node:
                        constrain.append(("1 ~R({},{}) 1 C({},{}) >= 1;".format(str(i), str(j), str(i), str(k-1))))
                        constrain.append(("1 ~R({},{}) 1 ~C({},{}) >= 1;".format(str(i), str(j), str(j), str(0))))
                        for m in range(k-1):
                            constrain.append(("1 ~R({},{}) 1 ~C({},{}) 1 C({},{}) >= 1;".format(str(i), str(j), str(j), str(m+1), str(i), str(m))))
                start_const +="1 S({}) ".format(str(i))
                finish_const+="1 T({}) ".format(str(i))
        #始点、終点の制約
        constrain.append((start_const + "= 1;"))
        constrain.append((finish_const+ "= 1;"))
        constrain.append(("1 S({}) = 1;".format(str(clus_start_node))))
        constrain.append(("1 T({}) = 1;".format(str(clus_finish_node))))

        t="t {} {}".format(str(clus_start_node),str(clus_finish_node))
        return constrain, str(NODE), str(EDGE), t, l, 0


    else:
        for i in range(len(tmp_path_list)-1):
            #print(use_path)
            use_path.append((tmp_path_list[i][1],tmp_path_list[i+1][0]))##確定パスのリストに追加(i,j)をリストに入れる
            c_k+=1
            step_list.append((tmp_path_list[i][1],tmp_path_list[i+1][1]))#ステップ数の大小関係をつけるべきリストに追加(t1,t2)をリストに入れる
        clus_start_node =tmp_path_list[0][0]
        clus_finish_node=tmp_path_list[-1][1]
        use_path_const=[]

        for path in use_path:
            i=path[0]
            j=path[1]
            if path[1] not in cluster_graph[path[0]]:
                O_list[i].append("1 R({},{}) ".format(str(i), str(j)))
                I_list[i].append("1 R({},{}) ".format(str(j), str(i)))
                O_list[j].append("1 R({},{}) ".format(str(j), str(i)))
                I_list[j].append("1 R({},{}) ".format(str(i), str(j)))
                use_path_const.append("1 ~R({},{}) 1 C({},{}) >= 1;".format(str(i), str(j), str(i), str(c_k-1)))
                use_path_const.append("1 ~R({},{}) 1 ~C({},{}) >= 1;".format(str(i), str(j), str(j), str(0)))
                use_path_const.append("1 ~R({},{}) 1 C({},{}) >= 1;".format(str(j), str(i), str(j), str(c_k-1)))
                use_path_const.append("1 ~R({},{}) 1 ~C({},{}) >= 1;".format(str(j), str(i), str(i), str(0)))
                min.append(" 1 R({},{})".format(str(i), str(j)))
                min.append(" 1 R({},{})".format(str(j), str(i)))
                EDGE+=2
                for m in range(c_k-1):
                    use_path_const.append("1 ~R({},{}) 1 ~C({},{}) 1 C({},{}) >= 1;".format(str(i), str(j), str(j), str(m+1), str(i), str(m)))
                    use_path_const.append("1 ~R({},{}) 1 ~C({},{}) 1 C({},{}) >= 1;".format(str(j), str(i), str(i), str(m+1), str(j), str(m)))

        ###制約生成#######################################
        #投射変数
        min="".join(min)
        constrain.append('min:'+min+';')
        #各ノードに対する制約
        for i in cluster_list[cluster_num]:
            #使わないノードの場合は制約を生成しない
            if i not in not_use_node:
                Ii="".join(I_list[i])
                Oi="".join(O_list[i])
                constrain.append((Ii+"<= 1;"))
                constrain.append(("d S({}) => ".format(str(i)) + Ii + "= 0;"))
                constrain.append(("d T({}) => ".format(str(i)) + Ii + "= 1;"))
                constrain.append((Oi+"<= 1;"))
                constrain.append(("d S({}) => ".format(str(i)) + Oi + "= 1;"))
                constrain.append(("d T({}) => ".format(str(i)) + Oi + "= 0;"))
                constrain.append(("d x({}) <= 1 ~S({}) 1 ~T({}) >= 2;".format(str(i), str(i), str(i))))
                constrain.append(("d x({}) => ".format(str(i)) + Ii + Oi.replace('1 ', '-1 ') + "= 0;"))
                for m in range(c_k-1):
                    constrain.append((cm_const[i][m]))

                #各辺に対する制約
                for j in cluster_graph[i]:
                    if j not in not_use_node:
                        constrain.append(("1 ~R({},{}) 1 C({},{}) >= 1;".format(str(i), str(j), str(i), str(c_k-1))))
                        constrain.append(("1 ~R({},{}) 1 ~C({},{}) >= 1;".format(str(i), str(j), str(j), str(0))))
                        for m in range(c_k-1):
                            constrain.append(("1 ~R({},{}) 1 ~C({},{}) 1 C({},{}) >= 1;".format(str(i), str(j), str(j), str(m+1), str(i), str(m))))

                start_const +="1 S({}) ".format(str(i))
                finish_const+="1 T({}) ".format(str(i))
        #始点、終点の制約
        constrain.append((start_const + "= 1;"))
        constrain.append((finish_const+ "= 1;"))
        constrain.append(("1 S({}) = 1;".format(str(clus_start_node))))
        constrain.append(("1 T({}) = 1;".format(str(clus_finish_node))))
        #繋いだパスが元のグラフにない場合に制約を作る
        for const in use_path_const:
            constrain.append((const))
        for path in use_path:
            i=path[0]
            j=path[1]
            constrain.append("1 R({},{}) = 1;".format(str(i), str(j)))
            constrain.append("1 R({},{}) = 0;".format(str(j), str(i)))
        #ステップ数の制約
        for path in step_list:
            t1=path[0]
            t2=path[1]
            for m in range(c_k-1):
                constrain.append(("1 ~C({},{}) 1 C({},{}) >= 1;".format(str(t2), str(m+1), str(t1), str(m))))

        l="l {}".format(c_k)
        t="t {} {}".format(str(clus_start_node),str(clus_finish_node))
        return constrain, str(NODE), str(EDGE), t, l, len(use_path)

#クラスタの制約を一から作る関数　（共通制約を利用しないパターン）
def cluster_const2(path_list,k,cluster_num):
    global cluster_list     #クラスタごとの集合　index:クラスタ番号 中身:クラスタに含まれるノード番号
    global cluster_graph    #全体のグラフからクラスタ連結パスを除去したグラフ
    global cluster_edge_2   #クラスタの連結パスの隣接リスト
    global n                #変数の数

    global start_node
    global finish_node
    global start_cluster
    global finish_cluster

    c_k=k
    tmp_path_list=list(path_list)
    not_use_node=[]
    use_path=[]
    step_list=[]
    I_list=[]
    O_list=[]
    min=[]
    start_const=""
    finish_const=""
    constrain=[]
    tmp_graph=copy.deepcopy(cluster_edge_2)

    NODE=n
    EDGE=0
    for i in cluster_list[cluster_num]:
        EDGE+=len(cluster_graph[i])
    l="l {}".format(k)

    for no_step in tmp_path_list:
        #入ってすぐ出るノードと端の始点終点を取り除く
        if no_step[0]==no_step[1]:
            i=no_step[0]
            tmp_path_list.remove(no_step)
            not_use_node.append(i)
            for j in tmp_graph[i]:
                tmp_graph[j].remove(i)
            tmp_graph[i]=[]
            #繋がりのあるノードからも消す

    if len(tmp_path_list)==0:
        return ["no_constrain"]###0ステップ

    elif len(tmp_path_list)==1:
        clus_start_node =tmp_path_list[0][0]
        clus_finish_node=tmp_path_list[0][1]

        ###制約生成#######################################
        #投射変数
        min="min:"
        I=["" for _ in range(n+1)]
        O=["" for _ in range(n+1)]
        for i in cluster_list[cluster_num]:
            for j in tmp_graph[i]:
                min += " 1 R({},{})".format(str(i),str(j))##パスを投射変数に追加
                I[i]+="1 R({},{})".format(str(j),str(i))
                O[i]+="1 R({},{})".format(str(i),str(j))
        constrain.append(min+';')
        for i in cluster_list[cluster_num]:
            if i not in not_use_node:
                Ii=I[i]
                Oi=O[i]
                constrain.append((Ii+"<= 1;"))
                constrain.append(("d S({}) => ".format(str(i)) + Ii + "= 0;"))
                constrain.append(("d T({}) => ".format(str(i)) + Ii + "= 1;"))
                constrain.append((Oi+"<= 1;"))
                constrain.append(("d S({}) => ".format(str(i)) + Oi + "= 1;"))
                constrain.append(("d T({}) => ".format(str(i)) + Oi + "= 0;"))
                constrain.append(("d x({}) <= 1 ~S({}) 1 ~T({}) >= 2;".format(str(i), str(i), str(i))))
                constrain.append(("d x({}) => ".format(str(i)) + Ii + Oi.replace('1 ', '-1 ') + "= 0;"))
                for m in range(k-1):
                    constrain.append("1 ~C({},{}) 1 C({},{}) >= 1;".format(str(i), str(m), str(i), str(m+1)))

                #各辺に対する制約
                for j in tmp_graph[i]:
                    if j not in not_use_node:
                        constrain.append(("1 ~R({},{}) 1 C({},{}) >= 1;".format(str(i), str(j), str(i), str(k-1))))
                        constrain.append(("1 ~R({},{}) 1 ~C({},{}) >= 1;".format(str(i), str(j), str(j), str(0))))
                        for m2 in range(k-1):
                            constrain.append(("1 ~R({},{}) 1 ~C({},{}) 1 C({},{}) >= 1;".format(str(i), str(j), str(j), str(m2+1), str(i), str(m2))))
                start_const +="1 S({}) ".format(str(i))
                finish_const+="1 T({}) ".format(str(i))

        #始点、終点の制約
        constrain.append((start_const + "= 1;"))
        constrain.append((finish_const+ "= 1;"))
        constrain.append(("1 S({}) = 1;".format(str(clus_start_node))))
        constrain.append(("1 T({}) = 1;".format(str(clus_finish_node))))

        t="t {} {}".format(str(clus_start_node),str(clus_finish_node))
        print(constrain)
        return constrain, str(NODE), str(EDGE), t, l, 0


    else:
        for i in range(len(tmp_path_list)-1):
            #print(use_path)
            use_path.append((tmp_path_list[i][1],tmp_path_list[i+1][0]))##確定パスのリストに追加(i,j)をリストに入れる
            c_k+=1
            step_list.append((tmp_path_list[i][1],tmp_path_list[i+1][1]))#ステップ数の大小関係をつけるべきリストに追加(t1,t2)をリストに入れる
        clus_start_node =tmp_path_list[0][0]
        clus_finish_node=tmp_path_list[-1][1]
        ###制約生成#######################################
        #投射変数
        min="min: "
        I=["" for _ in range(n+1)]
        O=["" for _ in range(n+1)]
        for i in cluster_list[cluster_num]:
            for j in tmp_graph[i]:
                min +="1 R({},{})".format(str(i),str(j))##パスを投射変数に追加
                I[i]+="1 R({},{})".format(str(j),str(i))
                O[i]+="1 R({},{})".format(str(i),str(j))
        for path in use_path:
            i=path[0]
            j=path[1]
            if j not in tmp_graph[i]:
                min +="1 R({},{})".format(str(i),str(j))##パスを投射変数に追加
                I[j]+="1 R({},{})".format(str(i),str(j))
                O[i]+="1 R({},{})".format(str(i),str(j))
            if i not in tmp_graph[j]:
                min +="1 R({},{})".format(str(j),str(i))##パスを投射変数に追加
                I[j]+="1 R({},{})".format(str(j),str(i))
                O[i]+="1 R({},{})".format(str(j),str(i))
        constrain.append(min+';')
        for i in cluster_list[cluster_num]:
            if i not in not_use_node:
                Ii=I[i]
                Oi=O[i]
                constrain.append((Ii+"<= 1;"))
                constrain.append(("d S({}) => ".format(str(i)) + Ii + "= 0;"))
                constrain.append(("d T({}) => ".format(str(i)) + Ii + "= 1;"))
                constrain.append((Oi+"<= 1;"))
                constrain.append(("d S({}) => ".format(str(i)) + Oi + "= 1;"))
                constrain.append(("d T({}) => ".format(str(i)) + Oi + "= 0;"))
                constrain.append(("d x({}) <= 1 ~S({}) 1 ~T({}) >= 2;".format(str(i), str(i), str(i))))
                constrain.append(("d x({}) => ".format(str(i)) + Ii + Oi.replace('1 ', '-1 ') + "= 0;"))
                for m in range(k-1):
                    constrain.append("1 ~C({},{}) 1 C({},{}) >= 1;".format(str(i), str(m), str(i), str(m+1)))

                #各辺に対する制約
                for j in tmp_graph[i]:
                    if j not in not_use_node:
                        constrain.append(("1 ~R({},{}) 1 C({},{}) >= 1;".format(str(i), str(j), str(i), str(k-1))))
                        constrain.append(("1 ~R({},{}) 1 ~C({},{}) >= 1;".format(str(i), str(j), str(j), str(0))))
                        for m2 in range(k-1):
                            constrain.append(("1 ~R({},{}) 1 ~C({},{}) 1 C({},{}) >= 1;".format(str(i), str(j), str(j), str(m2+1), str(i), str(m2))))
                start_const +="1 S({}) ".format(str(i))
                finish_const+="1 T({}) ".format(str(i))

        #始点、終点の制約
        constrain.append((start_const + "= 1;"))
        constrain.append((finish_const+ "= 1;"))
        constrain.append(("1 S({}) = 1;".format(str(clus_start_node))))
        constrain.append(("1 T({}) = 1;".format(str(clus_finish_node))))

        for path in use_path:
            i=path[0]
            j=path[1]
            constrain.append("1 R({},{}) = 1;".format(str(i), str(j)))
            constrain.append("1 R({},{}) = 0;".format(str(j), str(i)))
        #ステップ数の制約
        for path in step_list:
            t1=path[0]
            t2=path[1]
            for m in range(c_k-1):
                constrain.append(("1 ~C({},{}) 1 C({},{}) >= 1;".format(str(t2), str(m+1), str(t1), str(m))))

        l="l {}".format(str(c_k))
        t="t {} {}".format(str(clus_start_node),str(clus_finish_node))
        return constrain, str(NODE), str(EDGE), t, l, len(use_path)

def make_st_cluster_const(cluster_num):
    global cluster_graph
    global k

    tmp_graph=cluster_graph
    constrain=[]

    NODE=n
    EDGE=0
    for i in cluster_list[cluster_num]:
        EDGE+=len(cluster_graph[i])
    ###制約生成#######################################
    #投射変数
    min="min:"
    start_const =""
    finish_const=""
    start_end=""
    I=["" for _ in range(n+1)]
    O=["" for _ in range(n+1)]
    for i in cluster_list[cluster_num]:
        start_end +=" 1 S({}) 1 T({})".format(str(i),str(i))
        start_const +="1 S({}) ".format(str(i))
        finish_const+="1 T({}) ".format(str(i))
        for j in tmp_graph[i]:
            min += " 1 R({},{})".format(str(i),str(j))##パスを投射変数に追加
            I[i]+="1 R({},{}) ".format(str(j),str(i))
            O[i]+="1 R({},{}) ".format(str(i),str(j))

    min = min + start_end+';'
    constrain.append(min)

    for i in cluster_list[cluster_num]:
            Ii=I[i]
            Oi=O[i]
            constrain.append((Ii+"<= 1;"))
            constrain.append(("d S({}) => ".format(str(i)) + Ii + "= 0;"))
            constrain.append(("d T({}) => ".format(str(i)) + Ii + "= 1;"))
            constrain.append((Oi+"<= 1;"))
            constrain.append(("d S({}) => ".format(str(i)) + Oi + "= 1;"))
            constrain.append(("d T({}) => ".format(str(i)) + Oi + "= 0;"))
            constrain.append(("d x({}) <= 1 ~S({}) 1 ~T({}) >= 2;".format(str(i), str(i), str(i))))
            constrain.append(("d x({}) => ".format(str(i)) + Ii + Oi.replace('1 ', '-1 ') + "= 0;"))
            for m in range(k-1):
                constrain.append("1 ~C({},{}) 1 C({},{}) >= 1;".format(str(i), str(m), str(i), str(m+1)))

            #各辺に対する制約
            for j in tmp_graph[i]:
                #if j not in not_use_node:
                constrain.append(("1 ~R({},{}) 1 C({},{}) >= 1;".format(str(i), str(j), str(i), str(k-1))))
                constrain.append(("1 ~R({},{}) 1 ~C({},{}) >= 1;".format(str(i), str(j), str(j), str(0))))
                for m2 in range(k-1):
                    constrain.append(("1 ~R({},{}) 1 ~C({},{}) 1 C({},{}) >= 1;".format(str(i), str(j), str(j), str(m2+1), str(i), str(m2))))

    #始点、終点の制約
    constrain.append((start_const + "= 1;"))
    constrain.append((finish_const+ "= 1;"))

    l="l {}".format(str(k))
    #print(constrain)
    return constrain, str(NODE), str(EDGE), l, 0

def cal_result(a_list,b_list):
    result_list=[0]*(len(a_list)+len(b_list)-1)
    #print(result_list)
    if a_list==[]:
        return b_list
    if b_list==[]:
        return a_list
    for i,val_a in enumerate(a_list):
        for j,val_b in enumerate(b_list):
            result_list[i+j]+=val_a*val_b
    #print(result_list)

    return result_list

v_visit=[False]*(n+1)
def cal_cluster(v,cal_n,must,next,res_index=0):
    global v_visit

    result=[0]*(cal_n+1)
    for must_i in must:
        if v_visit[must_i]==False:
            break
        result[res_index]+=1
    v_visit[v]=True
    if v in must:
        for i in range(must.index(v)):
            if v_visit[must[i]] == False:
                break
            if cal_n >= 1:
                res_index+=1
                result[res_index]+=cal_cluster(next[v],cal_n,must,next,res_index)
    else:
        for u in cluster_edge_2[v]:
            if (v_visit[u]==False) and (cal_n>=1):
                result[res_index]+=cal_cluster(v,cal_n,must,next,res_index)
    v_visit[v]=False
    return result


############################################################
def printFeatures(G):

  # 特徴量の計算（リストとして返すので順番に注意）
  #  使わない物は適宜削除

  features = {}

  ## 変数の数
  #features['num_of_nodes'] = G.number_of_nodes()

  ## 辺の数
  #features['num_of_edge'] = G.number_of_edges()

  ## パスの長さ
  #features['path_length'] = length

  ## 連結成分数
  #features['num_of_connected_comp'] = nx.number_connected_components(G)

  ## 密度
  features['density'] = nx.density(G)

  ## 平均クラスタ係数
  #features['avg_clustering'] = nx.average_clustering(G)

  ## 推移性
  #features['transitivity'] = nx.transitivity(G)

  ## 媒介中心性
  #c = nx.betweenness_centrality(G, min(G.number_of_nodes(), 800), seed=1)
  #features['avg_between_cent'] = sum(c.values())/len(c)  # 平均
  #features['max_between_cent'] = max(c.values())         # 最大

  ## 固有ベクトル中心性
  #c = nx.eigenvector_centrality(G, max_iter=max(100000, 3*G.number_of_nodes()))
  #features['avg_eigenvector_cent'] = sum(c.values())/len(c)  # 平均
  #features['max_eigenvector_cent'] = max(c.values())         # 最大

  return features['density']


############################################################
''' メイン '''
''' メイン 制約の生成'''

if __name__ == '__main__':
    start_time=time.time()
    # 引数の解析
    args_obj = get_args()
    args = args_obj.parse_args()
    fname = args.input

    # ロード
    load(args.input)

    #sortnode=dijkstra(start_node, n)            #sortnodeにダイクストラでの距離順にノード番号を格納
    #dijkstra_simp(start_node,finish_node,n,k)  #内部表現のgraphを更新
    #del_endnode(n,start_node,finish_node)  #内部表現のgraphを末端ノードを削除して更新

    input_filename_base=(fname.split('/')[-1]).split('.')[0]
    print(input_filename_base+'.col')
    print('メイン出力')
    #クラスターに分割できる場合の処理
    if load_graph(graph,n,input_filename_base)[0]==True:
        proc_cluster(start_node,finish_node)
        #answer_listにクラスタ間を辿るパスを格納
        for i in positive_path:
            #print(i)
            find_path3(i)
        #print(len(positive_path))
        #sys.exit()

        #for i in answer_list:
        #    print(i)
        #クラスタ間を繋ぐパスを全体グラフから消去したグラフを作成
        make_cluster_graph()
        #共通する制約を生成
        make_common_const()

        tmp_proc_list=[]
        proc_list={}
        decrement_list=[]
        result=[]
        fname_list=[]
        #for i in check_list:
        #    print(i,check_list[i])
        # Parameters
        maxProcesses=multiprocessing.cpu_count()      # Maximum number of processes simultaneously invoked
        waitingSecs=0.1     # Waiting time in seconds

        commandLines=[]

        for i in check_list:
            #チェックするべきクラスタ内でのパス、そのパス上限、クラスタ番号を出力する
            #print(i,' : ',check_list[i])
            # 一時ファイルの生成
            fp = tempfile.NamedTemporaryFile(mode="r+",suffix=".pb",dir=".",delete=False)
            fname=fp.name.split('/')[-1]
            fname_list.append(fname)

            # 一時ファイルに書き込み
            count_info=cluster_const(i,check_list[i][0],check_list[i][1])

            #0ステップの場合
            if len(count_info)==1:
                fp.close()
                proc_list[i]=[1]
                continue

            #答えが0の場合
            elif len(count_info)==2:
                fp.close()
                proc_list[i]=[0]
                continue
            #0ステップでない場合
            for data in count_info[0]:
                fp.write(data+"\n")

            #print('pathcount呼び出し')
            #print(fname)#一時ファイル名の出力
            #proc="['./run_onepair_2.sh',{},{},{},{}]".format(fname,count_info[1],fname,count_info[2],fname,count_info[3],fname,count_info[4])
            proc=['./run_onepair_2.sh',fname,count_info[1],count_info[2],count_info[3],count_info[4]]
            commandLines.append(proc)
            #proc_echo = subprocess.Popen(['./run_onepair_2.sh',fname,count_info[1],count_info[2],count_info[3],count_info[4]], stdout=subprocess.PIPE)
            tmp_proc_list.append(i)
            decrement_list.append(int(count_info[-1]))
            # 一時ファイルを閉じる
            fp.close()

        numberOfCommand=len(commandLines)
        commandOutput = []
        # initialization of list of outputStrings
        for i in range(numberOfCommand):
            commandOutput.append(None)

        print("commandLines: {}".format(commandLines) )
        #print("commandOutput: {}".format(commandOutput) )

        processList=[]
        nextCommand=0
        allDone=False

        while not allDone:
            while len(processList) < maxProcesses and nextCommand < numberOfCommand:
                print("Invoking {}th command {}".format(nextCommand, commandLines[nextCommand]))
                processList.append( (nextCommand, subprocess.Popen(commandLines[nextCommand], stdout=subprocess.PIPE)) )
                nextCommand += 1

            time.sleep(waitingSecs)

            for (i,proc) in processList:
                if proc.poll() != None:
                    commandOutput[i] = proc.stdout.read().decode('utf=8')
                    #print("Finished {}th command, which results \"{}\"".format(i, commandOutput[i]))
                    del processList[ processList.index( (i,proc) ) ]
                    mozi_list=commandOutput[i].split('c')
                    result=[]
                    for g in mozi_list:
                      i_list=g.split(' ')
                      if "len" in i_list:
                          res=i_list.pop(-1).split('\n')[0]
                          result.append(int(res))
                    #UNSATの場合は空リストで帰ってくる
                    if result==[]:
                        proc_list[(tmp_proc_list[i])]=[0]
                    #SATの場合
                    else:
                        if decrement_list[i]==0:
                            proc_list[(tmp_proc_list[i])]=result
                        else:
                            for q in range(decrement_list[i]):
                                result.pop(0)
                            proc_list[(tmp_proc_list[i])]=result
            if processList == [] and nextCommand >= numberOfCommand:
                allDone = True

        #print("commandLines: {}".format(commandLines) )
        #print("commandOutput: {}".format(commandOutput) )

        #for i in fname_list:
        #    proc_echo = subprocess.Popen(['rm',i], stdout=subprocess.PIPE)

        counted_path=0
        #counted_path_list=[0]*(k+3)#counted_path_list パス数ごとに結果を格納
        #for i in proc_list:
        #    print(i,proc_list[i])
        for list in answer_list:
            max=list[0][0] #max　パスにおいてのクラスタで使えるパス数
            list_max=len(list)
            tmp_res_list=[[] for i in range(list_max)]
            for i in range(1,len(list)):
                #tmp_res_listにクラスタごとの解の数を入れる
                tmp_list=list[i]
                tmp_list.sort()
                tmp_list=tuple(tmp_list)
                tmp_res_list[i]=copy.deepcopy(proc_list[tmp_list])

            tmp_res=0
            use_path_num=k-max
            #計算結果の合計する計算
            #print(list,tmp_res_list)
            for i in range(len(list)-2):
                tmp_res_list[i+2]=cal_result(tmp_res_list[i+1],tmp_res_list[i+2])

            for i in range(max+1):
                #counted_path_list パス数ごとに結果を格納
                #counted_path_list[i+use_path_num]+=tmp_res_list[-1][i]
                #print(list,tmp_res_list)
                if i<len(tmp_res_list[-1]):
                    tmp_res+=tmp_res_list[-1][i]

            counted_path+=tmp_res
        #クラスタ一つのみのパス
        one_commandLines=[]
        for cluster_i in range(1,cluster_num+1):
            count_info=make_st_cluster_const(cluster_i)
            fp = tempfile.NamedTemporaryFile(mode="r+",suffix=".pb",dir=".",delete=False)
            fname=fp.name.split('/')[-1]
            fname_list.append(fname)
            # 一時ファイルに書き込み
            for data in count_info[0]:
                fp.write(data+"\n")
            #print('pathcount呼び出し')
            #print(fname)#一時ファイル名の出力
            proc=['./pathcount_beta_all.sh',fname,count_info[1],count_info[2],count_info[3]]
            one_commandLines.append(proc)
            #proc_echo = subprocess.Popen(['./pathcount_beta_all.sh',fname,count_info[1],count_info[2],count_info[3]], stdout=subprocess.PIPE)
            # 一時ファイルを閉じる
            fp.close()

        numberOfCommand=len(one_commandLines)
        #print(numberOfCommand)
        one_commandOutput = []
        # initialization of list of outputStrings
        for i in range(numberOfCommand):
            one_commandOutput.append(None)

        print("commandLines: {}".format(one_commandLines) )
        #print("commandOutput: {}".format(one_commandOutput) )

        one_processList=[]
        nextCommand=0
        allDone=False

        while not allDone:
            while len(one_processList) < maxProcesses and nextCommand < numberOfCommand:
                print("Invoking {}th command {}".format(nextCommand, one_commandLines[nextCommand]))
                one_processList.append( (nextCommand, subprocess.Popen(one_commandLines[nextCommand], stdout=subprocess.PIPE)) )
                nextCommand += 1

            time.sleep(waitingSecs)

            for (i,proc) in one_processList:
                if proc.poll() != None:
                    one_commandOutput[i] = proc.stdout.read().decode('utf=8')
                    #print("Finished {}th command, which results \"{}\"".format(i, commandOutput[i]))
                    del one_processList[ one_processList.index( (i,proc) ) ]
                    mozi_list=one_commandOutput[i].split('\n')
                    result=[]
                    for g in mozi_list:
                        i_list=g.split(' ')
                        if "complete" in i_list:
                            res=int(i_list.pop(-1))/2
                            print(str(res))
                        else:
                            pass
                    counted_path+=int(res)
            if one_processList == [] and nextCommand >= numberOfCommand:
                allDone = True

        for i in fname_list:
            proc_echo = subprocess.Popen(['rm',i], stdout=subprocess.PIPE)

        #print(counted_path_list)
        print('countedpath ',counted_path)

    #クラスター分割ができない場合の処理
    else:
        # 一時ファイルの生成
        #fp = tempfile.NamedTemporaryFile(mode="r+",suffix=".pb",dir=".",delete=False)
        #wp = tempfile.NamedTemporaryFile(mode="r+",suffix=".txt",dir=".",delete=False)
        #fname=fp.name.split('/')[-1]
        # 一時ファイルに書き込み
        #count_info=make_one_const()
        #for data in count_info[0]:
        #    fp.write(data+"\n")

        print('pathcount(origin_beta_all.sh)呼び出し')
        #print(fname)
        #proc_echo = subprocess.Popen(['echo',fname], stdout=subprocess.PIPE)
        #out1 = proc_echo.communicate()[0].decode('utf8', 'replace')
        #fname=out1
        #print(count_info)
        #proc_echo = subprocess.Popen(['./run_onepair_2.sh',fname,count_info[1],count_info[2],count_info[3],count_info[4]], stdout=subprocess.PIPE)
        proc_echo = subprocess.Popen(['./origin_beta_all.sh',input_filename], stdout=subprocess.PIPE)

        # 一時ファイルを閉じる
        out = proc_echo.communicate()[0].decode('utf8', 'replace')
        #print(out)
        mozi_list=out.split('\n')
        res=0
        for i in mozi_list:
            i_list=i.split(' ')
            if "complete" in i_list:
                res=i_list.pop(-1)
            else:
                pass
        print('countedpath ',res)
        #proc_echo_rm = subprocess.Popen(['rm',fname], stdout=subprocess.PIPE)

    for i in proc_list:
        if len(i)==1:
            print(i,proc_list[i])
    end_time=time.time()
    print('c counttime ',end_time-start_time)
