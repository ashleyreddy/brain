using System;
using System.Threading;
using Intel.RealSense;

namespace ConsoleApp2
{
    class Program
    {
        static void Main(string[] args)
        {

            Intel.RealSense.Config c;
            var cfg = new Intel.RealSense.Config();
            cfg.EnableStream(Stream.Pose, 1);

            Pipeline pipe = new Intel.RealSense.Pipeline();
            pipe.Start();
            FrameSet frames;
            Pose pose;

            //Intel.RealSense.Res

            while (true)
            {
                // Wait for the next set of frames from the camera
                frames = pipe.WaitForFrames();
                // Get a frame from the pose stream
                var f = frames.FirstOrDefault(Stream.Pose, Format.Any);

                pose = f.As<Intel.RealSense.PoseFrame>().PoseData;

                //!Must dispose of frames because its not managed code
                frames.Dispose();
                Console.WriteLine(pose.rotation.x);
                //Thread.Sleep(2000);

            }
        }
    }
}
