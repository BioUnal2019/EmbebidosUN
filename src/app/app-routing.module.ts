import { NgModule } from '@angular/core';
import { PreloadAllModules, RouterModule, Routes } from '@angular/router';

const routes: Routes = [
  {
    path: '',
    loadChildren: () => import('./tabs/tabs.module').then(m => m.TabsPageModule)
  },
  {
    path: 'doc-reg',
    loadChildren: () => import('./doc-reg/doc-reg.module').then( m => m.DocRegPageModule)
  },
  {
    path: 'doc-log',
    loadChildren: () => import('./doc-log/doc-log.module').then( m => m.DocLogPageModule)
  },
  {
    path: 'data',
    loadChildren: () => import('./data/data.module').then( m => m.DataPageModule)
  },
  {
    path: 'data-list',
    loadChildren: () => import('./data-list/data-list.module').then( m => m.DataListPageModule)
  }
];
@NgModule({
  imports: [
    RouterModule.forRoot(routes, { preloadingStrategy: PreloadAllModules })
  ],
  exports: [RouterModule]
})
export class AppRoutingModule {}
