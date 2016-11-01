using System;
using System.Collections.Generic;
using System.Configuration;
using System.Data;
using System.Linq;
using System.Threading.Tasks;
using System.Windows;
using Core;
using System.Collections.ObjectModel;

namespace AdvancedSubwayRoutePlanning
{
    /// <summary>
    /// App.xaml 的交互逻辑
    /// </summary>
    public partial class App : Application
    {
        private string[] args;
        public ObservableCollection<DisplayRouteUnit> DisplayRouteUnitList = new ObservableCollection<DisplayRouteUnit>();
        public bool IsShortestPlaning;

        private void AppStartup(object sender, StartupEventArgs e)
        {
            args = e.Args;
        }

        public string[] Args
        {
            get { return args; }
        }
    }
}
