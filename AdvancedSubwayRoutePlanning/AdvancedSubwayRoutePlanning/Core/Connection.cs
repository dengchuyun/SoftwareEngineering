namespace Core
{
    class Connection
    {
        public Station beginStation { get; }
        public Station endStation { get; }
        public string LineName { get; }
        public Connection(Station begin, Station end, string lineName)
        {
            this.beginStation = begin;
            this.endStation = end;
            this.LineName = lineName;
        }
        public override bool Equals(object obj)
        {
            if (obj == null || GetType() != obj.GetType())
            {
                return false;
            }
            return this.LineName == ((Connection)obj).LineName && this.beginStation.Equals(((Connection)obj).beginStation) && this.endStation.Equals(((Connection)obj).endStation);
        }
        public override int GetHashCode()
        {
            return (LineName + "|" + beginStation + "|" + endStation).GetHashCode();
        }
    }
}
