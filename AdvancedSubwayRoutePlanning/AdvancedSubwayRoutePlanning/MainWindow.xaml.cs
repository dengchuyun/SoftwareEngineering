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
        #region 字段区域

        private ObservableCollection<DisplayRouteUnit> displayRouteUnitList;
        private ObservableCollection<string> displayStationsName = new ObservableCollection<string>();
        private SubwayMap subwayMap;

        #endregion

        #region 构造区域

        public MainWindow()
        {
            InitializeComponent();
            this.subwayMap = BackgroundCore.GetBackgroundCore().SubwayMap;
            this.displayRouteUnitList = ((App)App.Current).DisplayRouteUnitList;
            this.listView_Route.ItemsSource = displayRouteUnitList;
            this.comboBox_StartStation.ItemsSource = displayStationsName;
            this.comboBox_EndStation.ItemsSource = displayStationsName;
            ((App)App.Current).IsShortestPlaning = (bool)radioButton_Shortest.IsChecked;

            BackgroundCore.GetBackgroundCore().SelectFunction(this, ((App)App.Current).Args);
        }

        #endregion

        #region 功能函数区域

        private void searchRoute()
        {
            if (subwayMap == null)
                return;

            string mode;

            if ((bool)radioButton_Shortest.IsChecked)
                mode = "-b";
            else
                mode = "-c";

            Cursor = Cursors.Wait;

            try
            {
                if (subwayMap.CurRoute != null)
                    subwayMap.CurRoute.Clear();
                displayRouteUnitList.Clear();
                subwayMap.SetStartStation(comboBox_StartStation.Text);
                subwayMap.SetEndStation(comboBox_EndStation.Text);
                subwayMap.CurRoute = subwayMap.GetDirections(mode);

                if (subwayMap.CurRoute.Count == 0)
                    throw new Exception("起始/终点站点相同！");

                displayRouteUnitList.Add(new DisplayRouteUnit(subwayMap.CurRoute[0].BeginStation.Name, subwayMap.CurRoute[0].LineName));
                foreach (Connection connection in (subwayMap.CurRoute))
                {
                    displayRouteUnitList.Add(new DisplayRouteUnit(connection.EndStation.Name, connection.LineName));
                }

                this.subwayGraph.ResetFlashIndex();
            }
            catch (Exception ex)
            {
                subwayMap.SetStartStation("");
                subwayMap.SetEndStation("");
                ErrorWindow errorWindow = new ErrorWindow();
                errorWindow.textBlock_Msg.Text = ex.Message;
                errorWindow.Show();
                return;
            }
            finally
            {
                Cursor = Cursors.Arrow;
                subwayGraph.InvalidateVisual();
            }
        }

        #endregion

        #region 事件区域

        private void button_Search_Click(object sender, RoutedEventArgs e)
        {
            searchRoute();
        }


        private void radioButton_Shortest_Click(object sender, RoutedEventArgs e)
        {
            ((App)App.Current).IsShortestPlaning = (bool)radioButton_Shortest.IsChecked;
            if (subwayMap != null && subwayMap.StartStation != null && subwayMap.EndStation != null)
                searchRoute();
        }

        private void radioButton_Least_Click(object sender, RoutedEventArgs e)
        {
            ((App)App.Current).IsShortestPlaning = (bool)radioButton_Shortest.IsChecked;
            if (subwayMap != null && subwayMap.StartStation != null && subwayMap.EndStation != null)
                searchRoute();
        }

        private void comboBox_StartStation_GotMouseCapture(object sender, MouseEventArgs e)
        {
            this.comboBox_StartStation.IsDropDownOpen = true;
        }

        private void comboBox_EndStation_GotMouseCapture(object sender, MouseEventArgs e)
        {
            this.comboBox_EndStation.IsDropDownOpen = true;
        }

        private void comboBox_StartStation_GotFocus(object sender, RoutedEventArgs e)
        {
            this.comboBox_StartStation.IsDropDownOpen = true;
        }

        private void comboBox_EndStation_GotFocus(object sender, RoutedEventArgs e)
        {
            this.comboBox_EndStation.IsDropDownOpen = true;
        }

        private void comboBox_Cities_DropDownOpened(object sender, EventArgs e)
        {
            ((Cities)this.stackPanel_FunctionArea.Resources["cities"]).Clear();
            foreach (string city in BackgroundCore.GetBackgroundCore().CityList)
                ((Cities)this.stackPanel_FunctionArea.Resources["cities"]).Add(city);
        }

        private void comboBox_Cities_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (this.comboBox_Cities.SelectedItem != null)
            {
                BackgroundCore.GetBackgroundCore().RefreshMap((string)this.comboBox_Cities.SelectedItem);
                subwayGraph.IsEnabled = true;
                subwayGraph.SetSubwayMap();
                this.subwayMap = BackgroundCore.GetBackgroundCore().SubwayMap;
                displayStationsName.Clear();
                foreach (Station station in subwayMap.Stations)
                {
                    displayStationsName.Add(station.Name);
                }
                subwayGraph.InvalidateVisual();
            }
        }

        #endregion
    }

    public class Cities : ObservableCollection<string>
    {
        public Cities()
        {
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
