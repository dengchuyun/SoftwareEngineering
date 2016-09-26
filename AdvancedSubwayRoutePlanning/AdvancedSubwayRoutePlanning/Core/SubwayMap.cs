using System.Collections.Generic;
using System.Collections;
using System;

namespace Core
{
    class SubwayMap
    {
        public List<Station> Stations { get; }
        public List<Connection> Connections { get; }
        public Hashtable SubwayLines { get; }
        private Hashtable map;
        public SubwayMap()
        {
            this.Stations = new List<Station>();
            this.Connections = new List<Connection>();
            this.SubwayLines = new Hashtable();
            this.map = new Hashtable();
        }

        public bool HasStation(string stationName)
        {
            return Stations.Exists(x => x.Name == stationName);
        }
        public void AddStation(string stationName, double x, double y)
        {
            if (!HasStation(stationName))
            {
                Station station = new Station(stationName, x, y);
                Stations.Add(station);
            }
        }
        public bool HasConnection(string begin, string end, string lineName)
        {
            return Connections.Exists(x => x.begin.Name == begin && x.end.Name == end && x.LineName == lineName);
        }
        public void AddConnection(string begin, string end, string lineName)
        {
            if (HasStation(begin) && HasStation(end))
            {
                if (!HasConnection(begin, end, lineName))
                {
                    Station beginStation = Stations.Find(x => x.Name == begin);
                    Station endStation = Stations.Find(x => x.Name == end);
                    Connection connection = new Connection(beginStation, endStation, lineName);
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

        public void AddSubwayLine(string name, string color)
        {
            SubwayLines.Add(name, new SubwayLine(name, color));
        }

        public List<Connection> GetDirections(string beginStationName, string endStationName, string mode)
        {
            int TRANSFERCOST =
            mode == "-b" ? 0 :
            mode == "-c" ? 10000 :
            -1;
            if (TRANSFERCOST < 0)
            {
                throw new AggregateException("模式错误");
            }
            if (!HasStation(beginStationName) || !HasStation(endStationName))
            {
                throw new ArgumentException("站点不存在！");
            }
            Station beginStation = Stations.Find(x => x.Name == beginStationName);
            Station endStation = Stations.Find(x => x.Name == endStationName);
            if (beginStation.Equals(endStation))
            {
                return new List<Connection>();
            }

            Queue<Station> queue = new Queue<Station>();
            Hashtable dis = new Hashtable();
            Hashtable pre = new Hashtable();
            Stations.ForEach(x => dis.Add(x, int.MaxValue));
            dis[beginStation] = 1;
            pre.Add(beginStation, new List<Connection>());
            queue.Enqueue(beginStation);
            while (queue.Count != 0)
            {
                Station now = queue.Dequeue();
                Connection comeConnection = ((List<Connection>)pre[now]).FindLast(x => true);
                ((List<Connection>)map[now]).ForEach(x =>
                {
                    int cost = 1 + (!now.Equals(beginStation) && comeConnection.LineName != x.LineName ? TRANSFERCOST : 0);
                    if ((int)dis[x.end] > (int)dis[now] + cost)
                    {
                        dis[x.end] = (int)dis[now] + cost;
                        pre[x.end] = new List<Connection>((List<Connection>)pre[now]);
                        ((List<Connection>)pre[x.end]).Add(x);
                        if (!queue.Contains(x.end))
                        {
                            queue.Enqueue(x.end);
                        }
                    }
                });
            }
            return (List<Connection>)pre[endStation];
        }
        public List<Station> GetLine(string lineName)
        {
            List<Station> line = new List<Station>();
            Connections.FindAll(x => x.LineName == lineName).ForEach(x =>
            {
                if (!line.Contains(x.begin))
                {
                    line.Add(x.begin);
                }
            });
            return line;
        }
    }
}
