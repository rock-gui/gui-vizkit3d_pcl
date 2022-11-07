#include "vizkit3d/Vizkit3DPlugin.hpp"
#include "PCLPointCloudVisualization.hpp"
#include "PolygonMeshVisualization.hpp"

namespace vizkit3d {
    class QtPluginVizkit : public vizkit3d::VizkitPluginFactory {
        Q_OBJECT
        Q_PLUGIN_METADATA(IID "rock.vizkit3d.pcl")
    private:
    public:

        QtPluginVizkit(){};

        /**
        * Returns a list of all available visualization plugins.
        * @return list of plugin names
        */
        virtual QStringList* getAvailablePlugins() const;

        virtual QObject* createPlugin(QString const& pluginName);
    };
    
    
}
