#include "Unity/IUnityInterface.h"
#import "UnityAppController.h"

extern void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginLoad(IUnityInterfaces* interfaces);
extern void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginUnload();

@interface TaichiUnityAppController : UnityAppController
{
}
- (void)shouldAttachRenderDelegate;
@end
@implementation TaichiUnityAppController
- (void)shouldAttachRenderDelegate {
    UnityRegisterRenderingPluginV5(&UnityPluginLoad, &UnityPluginUnload);
}
@end
IMPL_APP_CONTROLLER_SUBCLASS(TaichiUnityAppController);
