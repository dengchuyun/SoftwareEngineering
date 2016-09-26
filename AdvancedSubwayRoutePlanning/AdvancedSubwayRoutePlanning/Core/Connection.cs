namespace Core
{
    class Connection
    {
        public Station begin { get; }
        public Station end { get; }
        public string LineName { get; }
        public Connection(Station begin, Station end, string lineName)
        {
            this.begin = begin;
            this.end = end;
            this.LineName = lineName;
        }
        public override bool Equals(object obj)
        {
            if (obj == null || GetType() != obj.GetType())
            {
                return false;
            }
            return this.LineName == ((Connection)obj).LineName && this.begin.Equals(((Connection)obj).begin) && this.end.Equals(((Connection)obj).end);
        }
        public override int GetHashCode()
        {
            return (LineName + "|" + begin + "|" + end).GetHashCode();
        }
    }
}
