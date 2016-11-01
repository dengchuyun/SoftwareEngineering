using System;
using System.IO;
using System.Collections.Generic;
using System.Collections;
using AdvancedSubwayRoutePlanning;

namespace Core
{
    class BackgroundCore
    {
        public SubwayMap SubwayMap { get; private set; }
        public Printer Printer { get; }
        public List<string> CityList { get; }

        private Hashtable CityMap;
        private Loader loader;
        private List<Connection> route;
        private static BackgroundCore backgroundCore;

        private BackgroundCore()
        {
            loader = new Loader();
            CityMap = loader.LoadCityMap(@"Map/subway-list.xml");
            CityList = new List<string>();
            foreach (string k in CityMap.Keys)
                CityList.Add(k);
            foreach (string v in CityMap.Values)
                if (!File.Exists(@"Map/" + v))
                    throw new Exception();
            Printer = new Printer(System.Console.OpenStandardOutput());
        }

        public static BackgroundCore GetBackgroundCore()
        {
            try
            {
                if (backgroundCore == null) backgroundCore = new BackgroundCore();
            }
            catch (Exception)
            {
                throw new Exception("地图文件信息错误");
            }
            return backgroundCore;
        }

        public void RefreshMap(string CityName)
        {
            if (!CityList.Contains(CityName))
            {
                throw new ArgumentException("没有此城市");
            }
            SubwayMap = null;
            SubwayMap = loader.LoadSubwayMap(@"Map/" + (string)CityMap[CityName]);
        }

        public void SelectFunction(MainWindow mainWindow, string[] args)
        {
            mainWindow.Hide();
            if (args.Length == 1 && args[0] != "-g")
            {
                try
                {
                    RefreshMap(args[0]);
                }
                catch (Exception)
                {
                    Console.WriteLine("没有此城市");
                    goto CLOSE;
                }
                while (true)
                {
                    string input = System.Console.ReadLine();
                    if (input != "exit")
                    {
                        Printer.PrintSubwayLine(SubwayMap.GetLineByLabel(input));
                    }
                    else
                    {
                        goto CLOSE;
                    }
                }
            }
            else if (args.Length == 1 && args[0] == "-g")
            {
                mainWindow.Show();
                return;
            }
            else if (args.Length == 4)
            {
                try
                {
                    RefreshMap(args[0]);
                }
                catch (Exception)
                {
                    Console.WriteLine("没有此城市");
                    goto CLOSE;
                }
                SubwayMap.SetStartStation(args[2]);
                SubwayMap.SetEndStation(args[3]);
                try
                {
                    route = SubwayMap.GetDirections(args[1]);
                }
                catch (Exception)
                {
                    Console.WriteLine("始末站点错误");
                    goto CLOSE;
                }
                Printer.PrintDirections(route);
            }
            else
            {
                Printer.WriteLine("输入格式错误");
                goto CLOSE;
            }
CLOSE:
            mainWindow.Close();
        }
    }
}
