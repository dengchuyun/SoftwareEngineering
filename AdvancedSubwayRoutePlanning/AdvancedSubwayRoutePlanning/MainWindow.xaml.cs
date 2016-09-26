using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Collections;
using Core;

namespace AdvancedSubwayRoutePlanning
{
    /// <summary>
    /// MainWindow.xaml 的交互逻辑
    /// </summary>
    public partial class MainWindow : Window
    {
        private ObservableCollection<DisplayRouteUnit> displayRouteUnitList = new ObservableCollection<DisplayRouteUnit>();

        public MainWindow()
        {
            InitializeComponent();
        }

        private void searchRoute(object sender, RoutedEventArgs e)
        {
            string mode;

            if ((bool)radioButton_Shortest.IsChecked)
                mode = "-b";
            else
                mode = "-c";
            
            List<Connection> route = BackgroundCore.SubwayMap.GetDirections(comboBox_StartStation.Text, comboBox_EndStation.Text, mode);

            displayRouteUnitList.Clear();

            displayRouteUnitList.Add(new DisplayRouteUnit(route[0].beginStation.Name, route[0].LineName));
            foreach (Connection connection in route)
            {
                displayRouteUnitList.Add(new DisplayRouteUnit(connection.endStation.Name, connection.LineName));
            }

            listView_Route.ItemsSource = displayRouteUnitList;
        }
    }

    public class Cities : ObservableCollection<string>
    {
        public Cities()
        {
            Add("北京");
        }
    }

    public class DisplayRouteUnit
    {
        public string StationName { get; }
        public string LineName { get; }

        public DisplayRouteUnit(string stationName, string lineName)
        {
            this.StationName = stationName;
            this.LineName = lineName;
        }
    }

}
