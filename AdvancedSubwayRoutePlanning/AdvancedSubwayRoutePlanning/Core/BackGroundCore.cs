using System;
using System.Collections.Generic;
using AdvancedSubwayRoutePlanning;
namespace Core
{
    class BackgroundCore
    {
        private static Loader Loader = new Loader();
        private static Printer Printer = new Printer(System.Console.OpenStandardOutput());
        private static SubwayMap subwayMap;
        private static List<Connection> route = null;

        public BackgroundCore()
        {
            subwayMap = Loader.LoadFromXMLFile("subway.xml");
        }

        public static SubwayMap SubwayMap
        {
            get { return subwayMap; }
        }

        public static void SelectFunction(MainWindow mainWindow, string[] args)
        {
            try
            {
                mainWindow.Hide();

                if (args.Length == 0)
                {
                    while (true)
                    {
                        string input = System.Console.ReadLine();
                        if (input != "exit")
                        {
                            Printer.PrintSubwayLine(subwayMap.GetLine(input));
                        }
                        else
                        {
                            return;
                        }
                    }
                }
                else if (args.Length == 1 && args[0] == "-g")
                {
                    mainWindow.Show();
                }
                else if (args.Length == 3)
                {
                    route = subwayMap.GetDirections(args[1], args[2], args[0]);
                    Printer.PrintDirections(route);
                }
                else
                {
                    Printer.WriteLine("输入格式错误");
                }

            }
            catch (System.Exception e)
            {
                Console.WriteLine(e);
                mainWindow.Close();
                return;
            }
        }
    }
}
