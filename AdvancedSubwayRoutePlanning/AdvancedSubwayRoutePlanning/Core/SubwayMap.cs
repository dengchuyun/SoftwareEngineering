using System.Collections.Generic;
using System.Collections;
using System;

namespace Core
{
    class SubwayMap
    {
        public List<Station> Stations { get; }
        public List<Connection> Connections { get; }
        public List<SubwayLine> SubwayLines { get; }
        private Hashtable map;

        public Station StartStation { get; private set; }
        public Station EndStation { get; private set; }
        public List<Connection> CurRoute;

        public SubwayMap()
        {
            this.Stations = new List<Station>();
            this.Connections = new List<Connection>();
            this.SubwayLines = new List<SubwayLine>();
            this.map = new Hashtable();
            this.StartStation = null;
            this.EndStation = null;
        }

        public void SetStartStation(string name)
        {
            this.StartStation = Stations.Find(x => x.Name == name);
        }
        public void SetEndStation(string name)
        {
            this.EndStation = Stations.Find(x => x.Name == name);
        }
        public bool HasStation(string stationName)
        {
            return Stations.Exists(x => x.Name == stationName);
        }

        public void AddStation(string stationName, double x, double y, bool isTransfer)
        {
            if (!HasStation(stationName))
            {
                Station station = new Station(stationName, x, y,isTransfer);
                Stations.Add(station);
            }
        }

        public bool HasConnection(string begin, string end, string lineName)
        {
            return Connections.Exists(x => x.BeginStation.Name == begin && x.EndStation.Name == end && x.LineName == lineName);
        }

        public void AddConnection(string begin, string end, string lineName, int type)
        {
            if (HasStation(begin) && HasStation(end))
            {
                if (!HasConnection(begin, end, lineName))
                {
                    Station beginStation = Stations.Find(x => x.Name == begin);
                    Station endStation = Stations.Find(x => x.Name == end);
                    Connection connection = new Connection(beginStation, endStation, lineName,type);
                    Connections.Add(connection);
                    if (map.Contains(beginStation))
                    {
                        List<Connection> cons = (List<Connection>)map[beginStation];
                        cons.Add(connection);
                    }
                    else
                    {
                        List<Connection> cons = new List<Connection>();
                        cons.Add(connection);
                        map.Add(beginStation, cons);
                    }
                }
            }
            else
            {
                throw new System.ArgumentException("Invalid Connection!");
            }
        }

        internal int CountConnection(string c, string d)
        {
            Station a = Stations.Find(x=>x.Name==c);
            Station b = Stations.Find(x=>x.Name==d);
            return Connections.FindAll(x => x.BeginStation == a && x.EndStation == b).Count;
        }

        public void AddSubwayLine(string name, string label, string color)
        {
            SubwayLines.Add(new SubwayLine(name, label, color));
        }

        public void SortStations()
        {
            this.Stations.Sort((x, y) => x.Name.CompareTo(y.Name));
        }

        public List<Connection> GetDirections(string mode)
        {
            int transferCost =
            mode == "-b" ? 0 :
            mode == "-c" ? 10000 :
            -1;
            if (transferCost < 0) throw new AggregateException("模式错误");
            if (this.StartStation == null || this.EndStation == null) throw new Exception("站点未设置！");
            if (this.StartStation.Equals(this.EndStation)) return new List<Connection>();
            Queue<Station> queue = new Queue<Station>();
            Hashtable routeMap = new Hashtable();
            List<List<Connection>> initRouteList = new List<List<Connection>>();
            List<Connection> initRoute = new List<Connection>();
            Connection initConnection = new Connection(Stations.Find(x => x.Id == 0), Stations.Find(x => x.Id == 0), "*",-1);
            initRoute.Add(initConnection);
            initRouteList.Add(initRoute);
            routeMap.Add(this.StartStation, initRouteList);
            queue.Enqueue(this.StartStation);
            List<List<Connection>> tRouteList = null;
            List<Connection> tRoute = null;
            while (queue.Count != 0)
            {
                Station now = queue.Dequeue();
                List<List<Connection>> routeList = (List<List<Connection>>)routeMap[now];
                ((List<Connection>)map[now]).ForEach(line =>
                {
                    routeList.ForEach(route =>
                    {
                        if (!routeMap.Contains(line.EndStation)) routeMap.Add(line.EndStation, new List<List<Connection>>());
                        tRouteList = (List<List<Connection>>)routeMap[line.EndStation];
                        int alreadyCost = CountCost(mode, tRouteList.Find(x => true));
                        int nowCost = CountCost(mode, route) + (mode == "-b" ? 1 : mode == "-c" && route.Count != 1 && route.FindLast(x => true).LineName != line.LineName ? 1 : 0);
                        if (nowCost < alreadyCost)
                        {
                            tRouteList = new List<List<Connection>>();
                            tRoute = new List<Connection>(route);
                            tRoute.Add(line);
                            tRouteList.Add(tRoute);
                            routeMap[line.EndStation] = tRouteList;
                            if (!queue.Contains(line.EndStation)) queue.Enqueue(line.EndStation);
                        }
                        else if (nowCost == alreadyCost)
                        {
                            tRoute = new List<Connection>(route);
                            tRoute.Add(line);
                            if (CanBetter(tRouteList, tRoute, mode))
                            {
                                tRouteList.Add(tRoute);
                                if (!queue.Contains(line.EndStation)) queue.Enqueue(line.EndStation);
                            }
                        }
                    });
                });
            }

            if (!routeMap.Contains(this.EndStation)) throw new ArgumentException("站点不连通");

            tRouteList = (List<List<Connection>>)routeMap[this.EndStation];
            tRoute = tRouteList[0];
            tRouteList.ForEach(x =>
            {
                if (tRoute.Count > x.Count) tRoute = x;
                else if (tRoute.Count == x.Count) tRoute = CountCost("-c", tRoute) > CountCost("-c", x) ? x : tRoute;
            });
            tRoute.RemoveAt(0);
            return tRoute;
        }

        private bool CanBetter(List<List<Connection>> tRouteList, List<Connection> tRoute, string mode)
        {
            if (mode == "-b")
            {
                int k =  CountCost("-c", tRoute);
                return !tRouteList.Exists(x => CountCost("-c", x) < k);
            }
            else if (mode == "-c")
            {
                int k =  CountCost("-b", tRoute);
                return !tRouteList.Exists(x => x.FindLast(y => true).LineName == tRoute.FindLast(y => true).LineName);
            }
            return false;
        }

        private int CountCost(string mode, List<Connection> list)
        {
            if (list == null || list.Count == 0) return int.MaxValue;
            if (mode == "-b")
            {
                return list.Count;
            }
            else if (mode == "-c")
            {
                if (list.Count <= 2) return 0;
                int a = 0;
                string last = "*";
                list.ForEach(x => { if (x.LineName != last) { a++; last = x.LineName; } });
                return a;
            }
            else
                return -1;
        }

        public List<Station> GetLine(string lineName)
        {
            List<Station> line = new List<Station>();
            Connections.FindAll(x => x.LineName == lineName).ForEach(x =>
            {
                if (!line.Contains(x.BeginStation))
                {
                    line.Add(x.BeginStation);
                }
            });
            return line;
        }

        public Station GetStation(string stationName)
        {
            return Stations.Find(x => x.Name == stationName);
        }

        public List<Station> GetLineByLabel(string lineLabel)
        {
            List<Station> line = new List<Station>();
            SubwayLine subwayLine = SubwayLines.Find(w=>w.Label==lineLabel);
            if (subwayLine != null)
            {
                line = GetLine(subwayLine.Name);
            }
            return line;
        }
    }
}
