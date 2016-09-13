using System;
using System.Collections.Generic;
using System.Collections;
using System.IO;
using System.Text.RegularExpressions;
using DataStructure;

namespace SubwayRoutePlanning
{
    class Program
    {
        static void Main(string[] args)
        {
            Map map = new Map();
            RoutePlanning routePlanning = new RoutePlanning(map);
            routePlanning.PrintRoute(routePlanning.ShortestRoutePlanning("苹果园", "八角游乐园"));
            Console.WriteLine("------");
            routePlanning.PrintRoute(routePlanning.ShortestRoutePlanning("海淀五路居", "王府井"));
            Console.WriteLine("------");
            routePlanning.PrintRoute(routePlanning.ShortestRoutePlanning("昌平西山口", "次渠"));
            Console.ReadKey();
        }
    }

    class RoutePlanning
    {
        private int[,] floydAdjacencyMatrix;
        private int[,] pathMatrix;
        private Map map;

        public RoutePlanning(Map map)
        {
            floydAdjacencyMatrix = (int[,])map.StationsGraph.AdjacencyMatrix.Clone();
            pathMatrix = new int[floydAdjacencyMatrix.GetLength(0), floydAdjacencyMatrix.GetLength(1)];
            this.map = map;

            initFloyd();
        }

        private void initFloyd()
        {
            int stationVertexNum = floydAdjacencyMatrix.GetLength(0);

            for (int i = 0; i < stationVertexNum; i++)
            {
                for (int j = 0; j < stationVertexNum; j++)
                {
                    pathMatrix[i, j] = -1;
                }
            }

            for (int k = 0; k < stationVertexNum; k++)
            {
                for (int i = 0; i < stationVertexNum; i++)
                {
                    for (int j = 0; j < stationVertexNum; j++)
                    {
                        if (floydAdjacencyMatrix[i, k] == -1 || floydAdjacencyMatrix[k, j] == -1)
                            continue;
                        else if (floydAdjacencyMatrix[i, j] == -1 || (floydAdjacencyMatrix[i, j] > floydAdjacencyMatrix[i, k] + floydAdjacencyMatrix[k, j]))
                        {
                            floydAdjacencyMatrix[i, j] = floydAdjacencyMatrix[i, k] + floydAdjacencyMatrix[k, j];
                            pathMatrix[i, j] = k;
                        }
                    }
                }
            }
        }

        private void searchCompletedShortestRoute(int station1Index, int station2Index, List<string> shortestRoute)
        {
            int shortestRouteInsertIndex;
            int pathStationIndex = pathMatrix[station1Index, station2Index];

            if (pathStationIndex == -1)
                return;
            shortestRouteInsertIndex = shortestRoute.FindIndex((string stationName) => stationName == (string)map.Stations.GetKey(station2Index));
            shortestRoute.Insert(shortestRouteInsertIndex, (string)map.Stations.GetKey(pathStationIndex));
            searchCompletedShortestRoute(station1Index, pathStationIndex, shortestRoute);
            searchCompletedShortestRoute(pathStationIndex, station2Index, shortestRoute);
        }

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

        public void PrintRoute(List<string> route)
        {
            foreach (string stationName in route)
            {
                Console.WriteLine(stationName);
            }
        }
    }

    class Map
    {
        private Hashtable subwayLines = new Hashtable();
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
        public Hashtable SubwayLines
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
        private Hashtable subwayLines;

        public StationsGraph(SortedList stations, Hashtable subwayLines)
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
            SetAdjacencyMatrix();
        }
        //根据地铁路线信息设置
        private void SetAdjacencyMatrix()
        {
            foreach (SubwayLine subwayLine in subwayLines.Values)
            {
                int prevStationIndex;
                int curStationIndex;

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
