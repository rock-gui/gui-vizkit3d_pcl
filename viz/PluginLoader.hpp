#include "vizkit3d/Vizkit3DPlugin.hpp"
#include "PCLPointCloudVisualization.hpp"
#include "PolygonMeshVisualization.hpp"

namespace vizkit3d {
    class QtPluginVizkitPCL : public vizkit3d::VizkitPluginFactory {
        Q_OBJECT
        Q_PLUGIN_METADATA(IID "rock.vizkit3d.pcl")
    private:
    public:

        QtPluginVizkitPCL(){};

        /**
        * Returns a list of all available visualization plugins.
        * @return list of plugin names
        */
        virtual QStringList* getAvailablePlugins() const;

        virtual QObject* createPlugin(QString const& pluginName);
    };
    
    
}
