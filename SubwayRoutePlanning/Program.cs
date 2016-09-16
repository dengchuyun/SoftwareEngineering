using System;
using System.Collections.Generic;
using System.Collections;
using System.IO;
using System.Text.RegularExpressions;
using DataStructure;

namespace SubwayRoutePlanning
{
    //主程序类：负责命令行读入与整体控制
    class Program
    {
        static void Main(string[] args)
        {
            Map map = new Map();
            RoutePlanning routePlanning = new RoutePlanning(map);
            Console.WriteLine("------");
            routePlanning.PrintRoute(routePlanning.LeastTransferPlanning("南礼士路", "新街口"));
            //routePlanning.PrintRoute(routePlanning.ShortestRoutePlanning("南礼士路", "新街口"));
            //routePlanning.PrintRoute(routePlanning.LeastTransferPlanning("西局", "西单"));
            Console.ReadKey();
        }
    }
    //线路规划类：计算三种请求
    class RoutePlanning
    {
        private int stationVertexNum;
        private int[,] shortestRouteFloydAdjacencyMatrix, leastTransferFloydAdjacencyMatrix;
        private List<int>[,] shortestRouteMatrix, leastTransferMatrix;
        private Map map;

        public RoutePlanning(Map map)
        {
            this.map = map;
            stationVertexNum = map.Stations.Count;
            shortestRouteFloydAdjacencyMatrix = (int[,])map.StationsGraph.AdjacencyMatrix.Clone();
            shortestRouteMatrix = new List<int>[stationVertexNum, stationVertexNum];
            leastTransferFloydAdjacencyMatrix = (int[,])map.StationsGraph.OnlyTransferAdjacencyMatrix.Clone();
            leastTransferMatrix = new List<int>[stationVertexNum, stationVertexNum];

            initFloyd(shortestRouteFloydAdjacencyMatrix, shortestRouteMatrix);
            initFloyd(leastTransferFloydAdjacencyMatrix, leastTransferMatrix);
        }

        //为Floyd算法计算路径
        private void initFloyd(int[,] adjacencyMatrix, List<int>[,] pathMatrix)
        {
            for (int i = 0; i < stationVertexNum; i++)
                for (int j = 0; j < stationVertexNum; j++)
                {
                    pathMatrix[i, j] = new List<int>();
                    pathMatrix[i, j].Add(-1);
                }

            for (int k = 0; k < stationVertexNum; k++)
            {
                for (int i = 0; i < stationVertexNum; i++)
                {
                    if (k == i)
                        continue;
                    for (int j = 0; j < stationVertexNum; j++)
                    {
                        if (k == j)
                            continue;
                        if (adjacencyMatrix[i, k] == -1 || adjacencyMatrix[k, j] == -1)
                            continue;
                        else if (adjacencyMatrix[i, j] == -1 || (adjacencyMatrix[i, j] > adjacencyMatrix[i, k] + adjacencyMatrix[k, j]))
                        {
                            adjacencyMatrix[i, j] = adjacencyMatrix[i, k] + adjacencyMatrix[k, j];
                            pathMatrix[i, j].Clear();
                            pathMatrix[i, j].Add(k);
                        }
                        else if (adjacencyMatrix[i, j] == adjacencyMatrix[i, k] + adjacencyMatrix[k, j])
                            pathMatrix[i, j].Add(k);
                    }
                }
            }
        }

        //递归寻找Floyd算法中的完整最短路径
        private void searchCompletedShortestRoute(int station1Index, int station2Index, List<string> shortestRoute)
        {
            int shortestRouteInsertIndex;
            int pathStationIndex = shortestRouteMatrix[station1Index, station2Index][0];

            if (pathStationIndex == -1)
                return;
            shortestRouteInsertIndex = shortestRoute.FindIndex((string stationName) => stationName == (string)map.Stations.GetKey(station2Index));
            shortestRoute.Insert(shortestRouteInsertIndex, (string)map.Stations.GetKey(pathStationIndex));
            searchCompletedShortestRoute(station1Index, pathStationIndex, shortestRoute);
            searchCompletedShortestRoute(pathStationIndex, station2Index, shortestRoute);
        }

        //第一类需求：计算最短路径规划方案
        public List<string> ShortestRoutePlanning(string station1Name, string station2Name)
        {
            int station1Index, station2Index;
            List<string> shortestRoute = new List<string>();

            station1Index = map.Stations.IndexOfKey(station1Name);
            station2Index = map.Stations.IndexOfKey(station2Name);

            shortestRoute.Add(station1Name);
            shortestRoute.Add(station2Name);

            searchCompletedShortestRoute(station1Index, station2Index, shortestRoute);

            return shortestRoute;
        }

        //获取站点两侧换乘站点信息
        private List<string> getSideTransferStationsName(string stationName)
        {
            int stationIndex;
            SubwayLine subwayLine;
            List<string> sideTransferStationsName = new List<string>();
        
            subwayLine = (SubwayLine)map.SubwayLines[(((Station)map.Stations[stationName]).PlacedSubwayLineName[0])];
            stationIndex = subwayLine.InLineSubwayStationsNames.FindIndex((string stationNameInSubwayLine) => stationNameInSubwayLine == stationName);

            for (int i = stationIndex; i >= 0; i--)
            {
                if (((Station)map.Stations[subwayLine.InLineSubwayStationsNames[i]]).IsTransferStation)
                {
                    sideTransferStationsName.Add(subwayLine.InLineSubwayStationsNames[i]);
                    break;
                }
            }
            for (int i = stationIndex; i < subwayLine.InLineSubwayStationsNames.Count; i++)
            {
                if (((Station)map.Stations[subwayLine.InLineSubwayStationsNames[i]]).IsTransferStation)
                {
                    sideTransferStationsName.Add(subwayLine.InLineSubwayStationsNames[i]);
                    break;
                }
            }

            if (sideTransferStationsName.Count == 2 && sideTransferStationsName[0] == sideTransferStationsName[1])
                sideTransferStationsName.RemoveAt(1);

            return sideTransferStationsName;
        }

        //递归寻找Floyd算法中的只包含换乘站的最少换乘路径
        private void searchLeastTransferRoute(int station1Index, int station2Index, List<List<string>> leastTransferRoutes, int routeIndex)
        {
            int leastTransferRouteInsertIndex;
            int pathStationIndex;
            List<string> basicLeastTransferRoute = new List<string>(leastTransferRoutes[routeIndex].ToArray());

            if (station1Index == station2Index)
            {
                List<string> leastTransferRoute = new List<string>();
                leastTransferRoute.Add((string)map.Stations.GetKey(station1Index));
                leastTransferRoutes.Add(leastTransferRoute);
                return;
            }
            //实际递归部分
            for (int i = 0; i < leastTransferMatrix[station1Index, station2Index].Count; i++)
            {
                pathStationIndex = leastTransferMatrix[station1Index, station2Index][i];
                if (pathStationIndex == -1)
                    return;
                if (i >= 1)
                    leastTransferRoutes.Add(new List<string>(basicLeastTransferRoute.ToArray()));
                leastTransferRouteInsertIndex = leastTransferRoutes[routeIndex + i].FindIndex((string stationName) => stationName == (string)(map.Stations.GetKey(station2Index)));
                leastTransferRoutes[routeIndex + i].Insert(leastTransferRouteInsertIndex, (string)map.Stations.GetKey(pathStationIndex));
                searchLeastTransferRoute(station1Index, pathStationIndex, leastTransferRoutes, routeIndex + i);
                searchLeastTransferRoute(pathStationIndex, station2Index, leastTransferRoutes, routeIndex + i);
            }
        }

        //获取两站之间的所有站点（同一线路上）
        private List<string> getBetweenStations (string station1Name, string station2Name)
        {
            int station1SubwayLineIndex, station2SubwayLineIndex;
            string stationLinesName = GetStationsLineNames(station1Name, station2Name)[0];
            SubwayLine subwayLine = (SubwayLine)map.SubwayLines[stationLinesName];
            List<string> stationsInSubwauLine = new List<string>();

            station1SubwayLineIndex = subwayLine.InLineSubwayStationsNames.FindIndex((string stationName) => stationName == station1Name);
            station2SubwayLineIndex = subwayLine.InLineSubwayStationsNames.FindIndex((string stationName) => stationName == station2Name);

            if (station1SubwayLineIndex + 1 < station2SubwayLineIndex)
            {
                for (int i = station1SubwayLineIndex + 1; i < station2SubwayLineIndex; i++)
                    stationsInSubwauLine.Add(subwayLine.InLineSubwayStationsNames[i]);
                return stationsInSubwauLine;
            }
            else if (station1SubwayLineIndex > station2SubwayLineIndex + 1)
            {
                for (int i = station1SubwayLineIndex - 1; i > station2SubwayLineIndex; i--)
                    stationsInSubwauLine.Add(subwayLine.InLineSubwayStationsNames[i]);
                return stationsInSubwauLine;
            }

            return null;
        }

        //移除多个路径中较长的路径
        private List<List<string>> removeLongerRoute(List<List<string>> routes)
        {
            int leastStationsNum = routes[0].Count;
            List<List<string>> removedRoutes = new List<List<string>>();

            for (int i = 0; i < routes.Count; i++)
                if (leastStationsNum > routes[i].Count)
                    leastStationsNum = routes[i].Count;
            for (int i = 0; i < routes.Count; i++)
                if (routes[i].Count == leastStationsNum)
                    removedRoutes.Add(routes[i]);

            return removedRoutes;
        }
        //第二个需求：求解最少换乘路径
        public List<string> LeastTransferPlanning(string station1Name, string station2Name)
        {
            List<string> sideTransferStationsName1;
            List<string> sideTransferStationsName2;
            List<List<string>> leastTransferRoutes = new List<List<string>>();
            List<List<string>> completedLeastTransferRoutes = new List<List<string>>();
            int station1Index, station2Index;
            //两站均在同一地铁线上
            if (haveSameStationsLineNames(((Station)map.Stations[station1Name]).PlacedSubwayLineName, ((Station)map.Stations[station2Name]).PlacedSubwayLineName))
            {
                return ShortestRoutePlanning(station1Name, station2Name);
            }
            //两站位于不同地铁线上，需要进行换乘
            else
            {
                //获取只包含换乘站点的最短路径
                sideTransferStationsName1 = getSideTransferStationsName(station1Name);
                sideTransferStationsName2 = getSideTransferStationsName(station2Name);
                foreach (string sideStation1Name in sideTransferStationsName1)
                {
                    foreach (string sideStation2Name in sideTransferStationsName2)
                    {
                        station1Index = map.Stations.IndexOfKey(sideStation1Name);
                        station2Index = map.Stations.IndexOfKey(sideStation2Name);
                        leastTransferRoutes.Add(new List<string>());
                        leastTransferRoutes[leastTransferRoutes.Count - 1].Add(sideStation1Name);
                        if (sideStation1Name.Equals(sideStation2Name))
                            continue;
                        leastTransferRoutes[leastTransferRoutes.Count - 1].Add(sideStation2Name);
                        searchLeastTransferRoute(station1Index, station2Index, leastTransferRoutes, leastTransferRoutes.Count - 1);
                    }
                }
            }
            //去除换乘站点里换乘站点较多的路径
            leastTransferRoutes = removeLongerRoute(leastTransferRoutes);
            //获取完整的最少换乘线路
            for (int i = 0; i < leastTransferRoutes.Count; i++)
            {
                List<string> headBetweenStations;
                List<string> tailBetweenStations;

                completedLeastTransferRoutes.Add(new List<string>());
                completedLeastTransferRoutes[i].Add(leastTransferRoutes[i][0]);//添加第一个换乘站点
                for (int j = 1; j < leastTransferRoutes[i].Count; j++)
                {
                    List<string> betweenStations = getBetweenStations(leastTransferRoutes[i][j - 1], leastTransferRoutes[i][j]);
                    if (betweenStations == null)
                        break;
                    completedLeastTransferRoutes[i].AddRange(betweenStations);//插入中间站点
                    completedLeastTransferRoutes[i].Add(leastTransferRoutes[i][j]);//添加下一个换乘站点
                }

                headBetweenStations = getBetweenStations(station1Name, leastTransferRoutes[i][0]);
                if (headBetweenStations != null)
                {
                    headBetweenStations.AddRange(completedLeastTransferRoutes[i]);
                    completedLeastTransferRoutes[i] = headBetweenStations;
                }
                tailBetweenStations = getBetweenStations(leastTransferRoutes[i][leastTransferRoutes[i].Count - 1], station2Name);
                if (tailBetweenStations != null)
                    completedLeastTransferRoutes[i].AddRange(tailBetweenStations);

                completedLeastTransferRoutes[i].Insert(0, station1Name);
                completedLeastTransferRoutes[i].Add(station2Name);
            }

            completedLeastTransferRoutes = removeLongerRoute(completedLeastTransferRoutes);

            return completedLeastTransferRoutes[0];
        }

        //获取两站之间的地铁线路名（可能出现两个以上的线路）
        private List<string> GetStationsLineNames(string station1Name, string station2Name)
        {
            List<string> stationsLineNames = new List<string>();

            foreach (string subwayLineName1 in ((Station)map.Stations[station1Name]).PlacedSubwayLineName)
            {
                foreach (string subwayLineName2 in ((Station)map.Stations[station2Name]).PlacedSubwayLineName)
                {
                    if (subwayLineName1.Equals(subwayLineName2))
                        stationsLineNames.Add(subwayLineName2);
                }
            }

            return stationsLineNames;
        }

        //判断两个线路名集合中是否存在相同线路名
        private bool haveSameStationsLineNames(List<string> stationsLineNames1, List<string> stationsLineNames2)
        {
            foreach (string subwayLineName1 in (stationsLineNames1))
            {
                foreach (string subwayLineName2 in (stationsLineNames2))
                {
                    if (subwayLineName1.Equals(subwayLineName2))
                        return true;
                }
            }

            return false;
        }

        //根据规则打印路径信息
        public void PrintRoute(List<string> route)
        {
            Console.WriteLine(route[0]);
            for (int i = 2; i < route.Count; i++)
            {
                List<string> prevStationsLineNames = GetStationsLineNames(route[i - 2], route[i - 1]);
                List<string> curStationsLineNames = GetStationsLineNames(route[i - 1], route[i]);
                if (haveSameStationsLineNames(prevStationsLineNames, curStationsLineNames))
                    Console.WriteLine(route[i - 1]);
                else
                    Console.WriteLine(route[i - 1] + "换乘地铁" + curStationsLineNames[0]);
            }
            Console.WriteLine(route[route.Count - 1]);
        }
    }

    class Map
    {
        private SortedList subwayLines = new SortedList();
        private SortedList stations = new SortedList();
        private StationsGraph stationsGraph;

        public Map(string txtMapPath = "beijing-subway.txt")
        {
            LoadTxtMap(txtMapPath);
            stationsGraph = new StationsGraph(stations, subwayLines);
        }
        //加载文本格式地图
        private void LoadTxtMap(string txtMapPath)
        {
            try
            {
                using (StreamReader sr = new StreamReader(txtMapPath))
                {
                    string line;

                    while (true)
                    {
                        SubwayLine subwayLine = new SubwayLine();
                        subwayLine.IsCircle = false;
                        subwayLine.InLineSubwayStationsNames = new List<string>();
                        string[] stationsInfo;
                        //获取地铁线路名
                        if ((line = sr.ReadLine()) != null)
                        {
                            Match lineNameMatch = Regex.Match(line, @"^@([0-9a-zA-Z\u4E00-\u9FA5]+)");
                            if (lineNameMatch.Success)
                            {
                                subwayLine.IsCircle = true;
                                subwayLine.Name = lineNameMatch.Groups[1].Value;
                            }
                            else
                                subwayLine.Name = line;
                        }
                        else
                            return;

                        //获取此地铁线上所有地铁站名与地铁站所在地铁线路名
                        line = sr.ReadLine();
                        stationsInfo = Regex.Split(line, " ");
                        foreach (string stationInfo in stationsInfo)
                        {
                            //添加地铁站点名
                            Match stationNameMatch = Regex.Match(stationInfo, @"^[0-9a-zA-Z\u4E00-\u9FA5]+");
                            subwayLine.InLineSubwayStationsNames.Add(stationNameMatch.Value);
                            
                            //若地铁站为记录于系统中，则添加到stations中
                            if (!stations.Contains(stationNameMatch.Value))
                            {
                                Station station = new Station();
                                station.PlacedSubwayLineName = new List<string>();

                                station.Name = stationNameMatch.Value;

                                //添加所在地铁线路名
                                station.PlacedSubwayLineName.Add(subwayLine.Name);
                                MatchCollection subwayLineMatchCollection = Regex.Matches(stationInfo, @"\[([0-9a-zA-Z\u4E00-\u9FA5]+)\]");
                                foreach (Match m in subwayLineMatchCollection)
                                {
                                    station.IsTransferStation = true;
                                    station.PlacedSubwayLineName.Add(m.Groups[1].Value);
                                }

                                stations.Add(station.Name, station);
                            }
                        }

                        subwayLines.Add(subwayLine.Name, subwayLine);
                    }
                }
            }
            catch(Exception e)
            {
                Console.WriteLine("The file could not be read:");
                Console.WriteLine(e.Message);
            }

            return;
        }
        //返回地铁线路表
        public SortedList SubwayLines
        {
            get
            {
                return subwayLines;
            }
        }
        //返回地铁站点表
        public SortedList Stations
        {
            get
            {
                return stations;
            }
        }

        public StationsGraph StationsGraph
        {
            get
            {
                return stationsGraph;
            }
        }
    }
}

namespace DataStructure
{
    struct Station
    {
        public string Name;
        public bool IsTransferStation;
        public List<string> PlacedSubwayLineName;
    };
    
    struct SubwayLine
    {
        public string Name;
        public bool IsCircle;
        public List<string> InLineSubwayStationsNames;
    };
    
    class StationsGraph
    {
        private int stationVertexNum;
        private SortedList stationVertices;
        private int[,] adjacencyMatrix, onlyTransferAdjacencyMatrix;
        private SortedList subwayLines;

        public StationsGraph(SortedList stations, SortedList subwayLines)
        {
            this.stationVertexNum = stations.Count;
            this.stationVertices = stations;
            this.adjacencyMatrix = new int[stationVertexNum, stationVertexNum];
            this.onlyTransferAdjacencyMatrix = new int[stationVertexNum, stationVertexNum];
            this.subwayLines = subwayLines;
            for (int i = 0; i < stationVertexNum; i++)
            {
                for (int j = 0; j < stationVertexNum; j++)
                {
                    adjacencyMatrix[i, j] = -1;
                    onlyTransferAdjacencyMatrix[i, j] = -1;
                }
            }
            for (int i = 0; i < stationVertexNum; i++)
            {
                adjacencyMatrix[i, i] = 0;
                onlyTransferAdjacencyMatrix[i, i] = 0;
            }
            setAdjacencyMatrix();
            setOnlyTransferAdjacencyMatrix();
        }
        //根据地铁路线信息设置全站点邻接矩阵
        private void setAdjacencyMatrix()
        {
            foreach (SubwayLine subwayLine in subwayLines.Values)
            {
                int prevStationIndex, curStationIndex;

                for (int i = 1; i < subwayLine.InLineSubwayStationsNames.Count; i++)
                {
                    prevStationIndex = stationVertices.IndexOfKey(subwayLine.InLineSubwayStationsNames[i - 1]);
                    curStationIndex = stationVertices.IndexOfKey(subwayLine.InLineSubwayStationsNames[i]);
                    adjacencyMatrix[prevStationIndex, curStationIndex] = 1;
                    adjacencyMatrix[curStationIndex, prevStationIndex] = 1;
                }

                if (subwayLine.IsCircle)
                {
                    prevStationIndex = stationVertices.IndexOfKey(subwayLine.InLineSubwayStationsNames[subwayLine.InLineSubwayStationsNames.Count - 1]);
                    curStationIndex = stationVertices.IndexOfKey(subwayLine.InLineSubwayStationsNames[0]);
                    adjacencyMatrix[prevStationIndex, curStationIndex] = 1;
                    adjacencyMatrix[curStationIndex, prevStationIndex] = 1;
                }
            }

            return;
        }
        //根据地铁路线信息设置换乘站点邻接矩阵
        private void setOnlyTransferAdjacencyMatrix()
        {
            foreach (SubwayLine subwayLine in subwayLines.Values)
            {
                int prevTransferStationIndex = -1, curTransferStationIndex = -1;

                for (int i = 0; i < subwayLine.InLineSubwayStationsNames.Count; i++)
                {
                    curTransferStationIndex = stationVertices.IndexOfKey(subwayLine.InLineSubwayStationsNames[i]);
                    if (prevTransferStationIndex == -1 && ((Station)stationVertices.GetByIndex(curTransferStationIndex)).IsTransferStation)
                        prevTransferStationIndex = curTransferStationIndex;
                    else if (prevTransferStationIndex != -1 && ((Station)stationVertices.GetByIndex(curTransferStationIndex)).IsTransferStation)
                    {
                        onlyTransferAdjacencyMatrix[prevTransferStationIndex, curTransferStationIndex] = 1;
                        onlyTransferAdjacencyMatrix[curTransferStationIndex, prevTransferStationIndex] = 1;
                        prevTransferStationIndex = curTransferStationIndex;
                    }
                }

                if (subwayLine.IsCircle && prevTransferStationIndex != -1)
                {
                    for (int i = 0; i < subwayLine.InLineSubwayStationsNames.Count; i++)
                    {
                        curTransferStationIndex = stationVertices.IndexOfKey(subwayLine.InLineSubwayStationsNames[i]);
                        if (prevTransferStationIndex != -1 && ((Station)stationVertices.GetByIndex(curTransferStationIndex)).IsTransferStation)
                        {
                            onlyTransferAdjacencyMatrix[prevTransferStationIndex, curTransferStationIndex] = 1;
                            onlyTransferAdjacencyMatrix[curTransferStationIndex, prevTransferStationIndex] = 1;
                            break;
                        }
                    }
                }
            }

            return;
        }

        public int[,] AdjacencyMatrix
        {
            get
            {
                return adjacencyMatrix;
            }
        }

        public int[,] OnlyTransferAdjacencyMatrix
        {
            get
            {
                return onlyTransferAdjacencyMatrix;
            }
        }
    }
}
