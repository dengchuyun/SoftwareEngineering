using System;
using System.Collections.Generic;
using System.Configuration;
using System.Data;
using System.Linq;
using System.Threading.Tasks;
using System.Windows;

namespace AdvancedSubwayRoutePlanning
{
    /// <summary>
    /// App.xaml 的交互逻辑
    /// </summary>
    public partial class App : Application
    {
        private string[] args;

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
