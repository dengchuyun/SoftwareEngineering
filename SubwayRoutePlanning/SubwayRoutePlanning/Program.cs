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
            //命令行输入相关功能
            if (args.Length != 0)
                switch (args[0])
                {
                    //最短线路规划
                    case "-b":
                        if (args.Length == 3 && map.Stations.Contains(args[1]) && map.Stations.Contains(args[2]))
                            routePlanning.PrintRoute(routePlanning.ShortestRoutePlanning(args[1], args[2]));
                        else
                            Console.WriteLine("@@@最短路径规划站点参数有误@@@");
                        break;
                    //最少换乘线路规划
                    case "-c":
                        if (args.Length == 3 && map.Stations.Contains(args[1]) && map.Stations.Contains(args[2]))
                            routePlanning.PrintRoute(routePlanning.LeastTransferPlanning(args[1], args[2]));
                        else
                            Console.WriteLine("@@@最少换乘路径规划站点参数有误@@@");
                        break;
                    default:
                        Console.WriteLine("@@@命令类型错误@@@");
                        break;
                }
            //程序内输入相关功能
            else
            {
                string subwayLineName;
                while ((subwayLineName = Console.ReadLine()) != null)
                {
                    if (map.SubwayLines.Contains(subwayLineName))
                        foreach (string stationName in ((SubwayLine)map.SubwayLines[subwayLineName]).InLineSubwayStationsNames)
                            Console.WriteLine("*" + stationName);
                    else
                        Console.WriteLine("@@@线路输入错误@@@");
                }
            }
            Console.ReadKey();
        }
    }

    //线路规划类：计算三种请求
    class RoutePlanning
    {
        private int stationVertexNum;
        private int[,] floydAdjacencyMatrix;//最短路径数值矩阵
        private List<int>[,] routeMatrix;//最短路径矩阵

        private List<List<string>> leastTransferRoutes = new List<List<string>>();
        private int recursiveDepthLimit = -1;
        private Map map;

        public RoutePlanning(Map map)
        {
            this.map = map;
            stationVertexNum = map.Stations.Count;
            floydAdjacencyMatrix = (int[,])map.StationsGraph.AdjacencyMatrix.Clone();
            routeMatrix = new List<int>[stationVertexNum, stationVertexNum];

            initFloyd(floydAdjacencyMatrix, routeMatrix);
        }

        //为Floyd算法初始化路径相关矩阵(可计算多条线路)
        private void initFloyd(int[,] adjacencyMatrix, List<int>[,] routeMatrix)
        {
            for (int i = 0; i < stationVertexNum; i++)
                for (int j = 0; j < stationVertexNum; j++)
                {
                    routeMatrix[i, j] = new List<int>();
                    routeMatrix[i, j].Add(-1);
                }
            
            for (int k = 0; k < stationVertexNum; k++)
            {
                for (int i = 0; i < stationVertexNum; i++)
                {
                    if (k == i)//排除中间结点就是本身的情况
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
                            routeMatrix[i, j].Clear();
                            routeMatrix[i, j].Add(k);
                        }
                        else if (adjacencyMatrix[i, j] == adjacencyMatrix[i, k] + adjacencyMatrix[k, j])
                            routeMatrix[i, j].Add(k);
                    }
                }
            }
        }

        //递归寻找Floyd算法中的完整最短路径，不断查找两结点间的最短路径中间结点
        private void searchCompletedShortestRoute(int station1Index, int station2Index, List<string> shortestRoute)
        {
            int shortestRouteInsertIndex;
            int pathStationIndex = routeMatrix[station1Index, station2Index][0];
            //将两结点无中间结点状态作为递归基
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

        //递归寻找Floyd算法中的只包含换乘站的最少换乘路径
        private void searchLeastTransferRoute(int curRecursiveDepth, string curStationName, string curSubwayLineName, string targetStationName, List<string> notRecursiveSubwayLines, List<string> leastTransferRoute)
        {
            List<string> inLineSubwayStationsNames = ((SubwayLine)map.SubwayLines[curSubwayLineName]).InLineSubwayStationsNames;
            List<string> newNotRecursiveSubwayLines = new List<string>(notRecursiveSubwayLines.ToArray());
            List<string> newLeastTransferRoute = new List<string>(leastTransferRoute.ToArray());

            newNotRecursiveSubwayLines.Remove(curSubwayLineName);
            newLeastTransferRoute.Add(curStationName);
            //剪枝操作，保证递归深度不超过已知的最优状态
            if (recursiveDepthLimit != -1 && curRecursiveDepth > recursiveDepthLimit)
                return;

            if (inLineSubwayStationsNames.Contains(targetStationName))
            {
                recursiveDepthLimit = curRecursiveDepth;
                newLeastTransferRoute.Add(targetStationName);
                leastTransferRoutes.Add(newLeastTransferRoute);
                return;
            }

            foreach (string stationName in inLineSubwayStationsNames)
            {
                Station station = (Station)map.Stations[stationName];
                if (station.IsTransferStation)
                    foreach (string subwayLineName in station.PlacedSubwayLineName)
                        if (newNotRecursiveSubwayLines.Contains(subwayLineName))
                            searchLeastTransferRoute(curRecursiveDepth + 1, stationName, subwayLineName, targetStationName, newNotRecursiveSubwayLines, newLeastTransferRoute);
            }

            return;
        }

        //获取两站之间的所有站点（同一线路上），不存在站点时返回null
        private List<string> getBetweenStations (string station1Name, string station2Name)
        {
            int station1SubwayLineIndex, station2SubwayLineIndex;
            string stationLinesName = GetStationsLineNames(station1Name, station2Name)[0];
            SubwayLine subwayLine = (SubwayLine)map.SubwayLines[stationLinesName];
            List<string> stationsInSubwauLine = new List<string>();
            List<string> outerStationsInSubwauLine = new List<string>();

            station1SubwayLineIndex = subwayLine.InLineSubwayStationsNames.FindIndex((string stationName) => stationName == station1Name);
            station2SubwayLineIndex = subwayLine.InLineSubwayStationsNames.FindIndex((string stationName) => stationName == station2Name);

            if (station1SubwayLineIndex + 1 < station2SubwayLineIndex)
                for (int i = station1SubwayLineIndex + 1; i < station2SubwayLineIndex; i++)
                    stationsInSubwauLine.Add(subwayLine.InLineSubwayStationsNames[i]);
            else if (station1SubwayLineIndex > station2SubwayLineIndex + 1)
                for (int i = station1SubwayLineIndex - 1; i > station2SubwayLineIndex; i--)
                    stationsInSubwauLine.Add(subwayLine.InLineSubwayStationsNames[i]);
            //考虑到回环的地铁线路，需要另一方向的路线记录
            if (subwayLine.IsCircle)
            {
                if (station1SubwayLineIndex < station2SubwayLineIndex && station1SubwayLineIndex + subwayLine.InLineSubwayStationsNames.Count - station2SubwayLineIndex > 0)
                {
                    for (int i = station1SubwayLineIndex - 1; i >= 0; i--)
                        outerStationsInSubwauLine.Add(subwayLine.InLineSubwayStationsNames[i]);
                    for (int i = subwayLine.InLineSubwayStationsNames.Count - 1; i > station2SubwayLineIndex; i--)
                        outerStationsInSubwauLine.Add(subwayLine.InLineSubwayStationsNames[i]);
                }
                else if (station1SubwayLineIndex > station2SubwayLineIndex && station2SubwayLineIndex + subwayLine.InLineSubwayStationsNames.Count - station1SubwayLineIndex > 0)
                {
                    for (int i = station1SubwayLineIndex + 1; i < subwayLine.InLineSubwayStationsNames.Count; i++)
                        outerStationsInSubwauLine.Add(subwayLine.InLineSubwayStationsNames[i]);
                    for (int i = 0; i < station2SubwayLineIndex; i++)
                        outerStationsInSubwauLine.Add(subwayLine.InLineSubwayStationsNames[i]);
                }
            }
            //返回路径最小的间隔站点信息
            if (stationsInSubwauLine.Count == 0 && outerStationsInSubwauLine.Count == 0)
                return null;
            else if (stationsInSubwauLine.Count != 0 && outerStationsInSubwauLine.Count == 0)
                return stationsInSubwauLine;
            else if (stationsInSubwauLine.Count != 0 && outerStationsInSubwauLine.Count == 0)
                return outerStationsInSubwauLine;
            else
            {
                if (stationsInSubwauLine.Count <= outerStationsInSubwauLine.Count)
                    return stationsInSubwauLine;
                else
                    return outerStationsInSubwauLine;
            }     
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
            List<List<string>> completedLeastTransferRoutes = new List<List<string>>();
            List<string> notRecursiveSubwayLines = new List<string>();
            //初始化搜索参数
            leastTransferRoutes = new List<List<string>>();
            recursiveDepthLimit = -1;
            foreach (SubwayLine subwayLine in map.SubwayLines.Values)
                notRecursiveSubwayLines.Add(subwayLine.Name);
            //获取初步的换乘线路
            foreach (string subwayStationsName in ((Station)map.Stations[station1Name]).PlacedSubwayLineName)
                searchLeastTransferRoute(0, station1Name, subwayStationsName, station2Name, notRecursiveSubwayLines, new List<string>());
            //筛选出最少换乘路线
            leastTransferRoutes = removeLongerRoute(leastTransferRoutes);
            //获取完整的最少换乘线路
            for (int i = 0; i < leastTransferRoutes.Count; i++)
            {
                completedLeastTransferRoutes.Add(new List<string>());
                //添加第一个站点
                completedLeastTransferRoutes[i].Add(leastTransferRoutes[i][0]);
                for (int j = 1; j < leastTransferRoutes[i].Count; j++)
                {
                    List<string> betweenStations = getBetweenStations(leastTransferRoutes[i][j - 1], leastTransferRoutes[i][j]);
                    if (betweenStations != null)
                        completedLeastTransferRoutes[i].AddRange(betweenStations);
                    //插入中间站点并添加下一个换乘站点
                    completedLeastTransferRoutes[i].Add(leastTransferRoutes[i][j]);
                }
            }
            //筛选出最少换乘路线中最短路径
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
            Console.WriteLine(route.Count);
            Console.WriteLine(route[0]);
            for (int i = 2; i < route.Count; i++)
            {
                List<string> prevStationsLineNames = GetStationsLineNames(route[i - 2], route[i - 1]);
                List<string> curStationsLineNames = GetStationsLineNames(route[i - 1], route[i]);
                if (route[i - 2].Equals("四惠") && route[i - 1].Equals("四惠东"))
                    Console.WriteLine(route[i - 1] + "换乘地铁八通线");
                else if (route[i - 2].Equals("四惠东") && route[i - 1].Equals("四惠"))
                    Console.WriteLine(route[i - 1] + "换乘地铁1号线");
                else
                {
                    if (haveSameStationsLineNames(prevStationsLineNames, curStationsLineNames))
                        Console.WriteLine(route[i - 1]);
                    else
                        Console.WriteLine(route[i - 1] + "换乘地铁" + curStationsLineNames[0]);
                }
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
                            else if (Regex.Match(line, @"^([0-9a-zA-Z\u4E00-\u9FA5]+)").Success)
                                subwayLine.Name = line;
                            else
                                throw new Exception("地铁线路名格式有误");
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
                            if (!stationNameMatch.Success)
                                throw new Exception("地铁站名格式有误");
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
                                    if (!m.Success)
                                        throw new Exception("地铁站名格式有误");
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
                Console.WriteLine("文件无法读取");
                Console.WriteLine(e.Message);
                System.Environment.Exit(0);
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
        private int[,] adjacencyMatrix;
        private SortedList subwayLines;

        public StationsGraph(SortedList stations, SortedList subwayLines)
        {
            this.stationVertexNum = stations.Count;
            this.stationVertices = stations;
            this.adjacencyMatrix = new int[stationVertexNum, stationVertexNum];
            this.subwayLines = subwayLines;
            for (int i = 0; i < stationVertexNum; i++)
            {
                for (int j = 0; j < stationVertexNum; j++)
                {
                    adjacencyMatrix[i, j] = -1;
                }
            }
            for (int i = 0; i < stationVertexNum; i++)
            {
                adjacencyMatrix[i, i] = 0;
            }
            setAdjacencyMatrix();
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
        
        public int[,] AdjacencyMatrix
        {
            get
            {
                return adjacencyMatrix;
            }
        }
    }
}