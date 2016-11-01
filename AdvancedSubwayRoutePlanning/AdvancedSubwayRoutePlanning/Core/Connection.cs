namespace Core
{
    public class Connection
    {
        public Station BeginStation { get; }
        public Station EndStation { get; }
        public string LineName { get; }
        private int type;

        public int Type
        {
            get { return type; }
            set
            {
                if (-1 <= value && value <= 2)
                    type = value;
            }
        }

        public Connection(Station beginStationName, Station endStationName, string lineName, int type)
        {
            this.BeginStation = beginStationName;
            this.EndStation = endStationName;
            this.LineName = lineName;
            this.type = type;
        }

        public override bool Equals(object obj)
        {
            if (obj == null || GetType() != obj.GetType())
            {
                return false;
            }
            return this.LineName == ((Connection)obj).LineName && this.BeginStation.Equals(((Connection)obj).BeginStation) && this.EndStation.Equals(((Connection)obj).EndStation);
        }

        public override int GetHashCode()
        {
            return (LineName + "|" + BeginStation + "|" + EndStation).GetHashCode();
        }
    }
}
